/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 *
 * Dongle main: ESB PRX → USB HID forwarding with simple pairing state machine.
 *
 * Pairing state:
 *   UNPAIRED  — no stored peer; button press arms pairing
 *   PAIRING   — arming: will respond to next BEACON with PAIR_REQ in ACK
 *   VERIFYING — stored peer, no verified session this boot; HID suppressed
 *               until a matching VERIFY_REQ arrives
 *   PAIRED    — verified session; HID reports on pipe 1 forwarded to USB HID
 *
 * Mutual verification: keyboard's FICR device_id is captured in PAIR_RESP
 * and persisted. On reconnect the dongle boots into VERIFYING and only
 * promotes to PAIRED when VERIFY_REQ arrives from the stored peer. Strangers
 * get DISCONNECT and are ignored.
 *
 * Button (P0.29, active low):
 *   short press when UNPAIRED → arm pairing (PAIRING)
 *   hold 5s    when PAIRED   → forget peer (UNPAIRED)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/settings/settings.h>
#include <hal/nrf_ficr.h>
#include <string.h>

#include <zmk_esb/protocol.h>
#include "esb_prx.h"
#include "usb_hid.h"
#include "led_status.h"
#include "shell_relay.h"
#include "bench.h"
#if IS_ENABLED(CONFIG_DONGLE_CHANNEL_HOP)
#include "channel_hop_dongle.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dongle_main, LOG_LEVEL_INF);

#define SETTINGS_KEY  "dongle/paired"

typedef enum {
    STATE_UNPAIRED,
    STATE_PAIRING,
    STATE_VERIFYING,
    STATE_PAIRED,
} dongle_state_t;

/* Persisted pairing record. A length mismatch on load (e.g. the pre-mutual
 * single-byte format) silently wipes it so we boot as UNPAIRED. */
struct dongle_paired_rec {
    uint8_t paired;
    uint8_t peer_device_id[6];
} __attribute__((__packed__));

static dongle_state_t m_state = STATE_UNPAIRED;
static bool m_has_peer;
static uint8_t m_device_id[6];
static uint8_t m_peer_device_id[6];

static void build_device_id(void) {
    const uint32_t lo = nrf_ficr_deviceid_get(NRF_FICR, 0);
    const uint32_t hi = nrf_ficr_deviceid_get(NRF_FICR, 1);
    memcpy(&m_device_id[0], &lo, 4);
    memcpy(&m_device_id[4], &hi, 2);
}

#define KB_REPORT_LEN       8
#define CONSUMER_REPORT_LEN 2
#define MOUSE_REPORT_LEN    9

static int settings_load_cb(const char *name, size_t len, const settings_read_cb read_cb, void *cb_arg) {
    if (len != sizeof(struct dongle_paired_rec)) {
        LOG_WRN("unexpected settings record length %zu, dropping", len);
        return 0;
    }
    struct dongle_paired_rec rec;
    read_cb(cb_arg, &rec, sizeof(rec));
    m_has_peer = (rec.paired == 1);
    memcpy(m_peer_device_id, rec.peer_device_id, sizeof(m_peer_device_id));
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(dongle, "dongle", NULL, settings_load_cb, NULL, NULL);

static void save_paired(const bool paired) {
    struct dongle_paired_rec rec = { .paired = paired ? 1 : 0 };
    if (paired) {
        memcpy(rec.peer_device_id, m_peer_device_id, sizeof(rec.peer_device_id));
    }
    settings_save_one(SETTINGS_KEY, &rec, sizeof(rec));
    m_has_peer = paired;
    if (!paired) {
        memset(m_peer_device_id, 0, sizeof(m_peer_device_id));
    }
    led_status_set_paired(paired);
}

static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_NODELABEL(pair_btn), gpios);
static struct gpio_callback btn_cb_data;
static struct k_work btn_work;
static struct k_work_delayable long_press_work;
static bool btn_was_pressed;
static bool long_press_fired;

static void long_press_work_fn(struct k_work *w) {
    long_press_fired = true;
    if (m_state == STATE_PAIRED || m_state == STATE_VERIFYING) {
        LOG_INF("Button held 5s: forget peer, UNPAIRED");
        save_paired(false);
        m_state = STATE_UNPAIRED;
        led_status_set_armed(false);
#if IS_ENABLED(CONFIG_DONGLE_CHANNEL_HOP)
        channel_hop_dongle_set_paired(false);
#endif
        /* Tell the keyboard to unpair. Queue one DISCONNECT ACK payload;
         * it will be delivered on the next keyboard TX. Don't rely on the
         * HID_REPORT handler for this — see below. */
        struct esb_pkt_disconnect d = { .type = ESB_PKT_DISCONNECT };
        esb_prx_queue_ack(1, (uint8_t *)&d, sizeof(d));
    }
}

static void btn_work_fn(struct k_work *w) {
    const bool pressed = (gpio_pin_get_dt(&btn) == 1);
    if (pressed && !btn_was_pressed) {
        btn_was_pressed = true;
        long_press_fired = false;
        k_work_schedule(&long_press_work, K_MSEC(CONFIG_DONGLE_UNPAIR_HOLD_MS));
    } else if (!pressed && btn_was_pressed) {
        btn_was_pressed = false;
        k_work_cancel_delayable(&long_press_work);
        if (!long_press_fired && m_state == STATE_UNPAIRED) {
            LOG_INF("Button short press: arm pairing (UNPAIRED -> PAIRING)");
            m_state = STATE_PAIRING;
            led_status_set_armed(true);
        } else if (!long_press_fired && m_state == STATE_PAIRED) {
            LOG_INF("Button short press: request shell relay");
            shell_relay_request();
        }
        
    }
}

static void btn_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_work_submit(&btn_work);
}

static void on_esb_rx(const uint8_t pipe, const uint8_t *data, const uint8_t len) {
    if (len < 1) {
        return;
    }

    led_status_mark_rx();
    LOG_DBG("packet type %d", data[0]);

#if IS_ENABLED(CONFIG_DONGLE_CHANNEL_HOP)
    /* Cooperative-hop OFFER must be parsed BEFORE note_rx_active —
     * the rejection logic in on_rx_offer inspects m_speculative_prev
     * and m_rollback_active before they get cleared by note_rx_active's
     * confirm_speculative_if_any / exit_rollback. After the parse,
     * the OFFER counts as any other non-IDLE packet and rearms the
     * watchdog via the normal note_rx_active path below. */
    if (data[0] == ESB_PKT_HOP_OFFER) {
        channel_hop_dongle_on_rx_offer(data, len);
    }

    /* Drive peer-idle tracking from the RX path. IDLE specifically flags
     * the peer as idle and disarms the silence watchdog; every other
     * packet type (including control packets like VERIFY_REQ and
     * CHANNEL_HOP_PROPOSAL) means the peer is still transmitting, so
     * rearm the watchdog. */
    if (data[0] == ESB_PKT_IDLE) {
        channel_hop_dongle_note_rx_idle();
    } else {
        channel_hop_dongle_note_rx_active();
    }
#endif

    switch (data[0]) {
    case ESB_PKT_BEACON:
        if (m_state == STATE_PAIRING) {
            LOG_INF("BEACON rx in PAIRING: queuing PAIR_REQ in ACK");
            struct esb_pkt_pair_req req = { .type = ESB_PKT_PAIR_REQ };
            memcpy(req.device_id, m_device_id, sizeof(req.device_id));
            esb_prx_queue_ack(0, (uint8_t *)&req, sizeof(req));
        }
        break;

    case ESB_PKT_PAIR_RESP:
        if (m_state == STATE_PAIRING && len >= sizeof(struct esb_pkt_pair_resp)) {
            const struct esb_pkt_pair_resp *pkt = (const void *)data;
            memcpy(m_peer_device_id, pkt->device_id, sizeof(m_peer_device_id));
            LOG_INF("PAIR_RESP: paired with %02X%02X%02X%02X%02X%02X",
                    m_peer_device_id[0], m_peer_device_id[1], m_peer_device_id[2],
                    m_peer_device_id[3], m_peer_device_id[4], m_peer_device_id[5]);
            save_paired(true);
            m_state = STATE_PAIRED;
            led_status_set_armed(false);
#if IS_ENABLED(CONFIG_DONGLE_CHANNEL_HOP)
            channel_hop_dongle_set_paired(true);
#endif
            if (IS_ENABLED(CONFIG_DONGLE_SHELL_RELAY_ALWAYS_ON)) {
                shell_relay_request();
            }
        }
        break;

    case ESB_PKT_HID_REPORT: {
        /* STATE_VERIFYING deliberately drops HID — a stranger keyboard that
         * skips the verify handshake must not reach the host. When we're in
         * VERIFYING but the keyboard is still sending HID (i.e. it's stuck in
         * CONNECTED from a previous session because we rebooted), queue a
         * RESYNC so it falls back to VERIFYING and re-runs the handshake. */
        if (m_state == STATE_VERIFYING) {
            struct esb_pkt_resync r = { .type = ESB_PKT_RESYNC };
            esb_prx_queue_ack(1, (uint8_t *)&r, sizeof(r));
            break;
        }
        if (m_state != STATE_PAIRED || len < 2) {
            break;
        }

        const struct esb_pkt_hid_report *pkt = (const void *)data;
        uint8_t report_id;
        uint8_t report_len;

        switch (pkt->report_type) {
        case ESB_REPORT_KEYBOARD:
            report_id  = CONFIG_DONGLE_HID_REPORT_ID_KB;
            report_len = KB_REPORT_LEN;
            break;
        case ESB_REPORT_CONSUMER:
            report_id  = CONFIG_DONGLE_HID_REPORT_ID_CONSUMER;
            report_len = CONSUMER_REPORT_LEN;
            break;
        case ESB_REPORT_MOUSE:
            report_id  = CONFIG_DONGLE_HID_REPORT_ID_MOUSE;
            report_len = MOUSE_REPORT_LEN;
            break;
        default:
            return;
        }

        if (pkt->report_type == ESB_REPORT_KEYBOARD) {
            LOG_DBG("RX KB: mod=%02X k=%02X %02X %02X %02X %02X %02X",
                    pkt->data[0], pkt->data[2], pkt->data[3], pkt->data[4],
                    pkt->data[5], pkt->data[6], pkt->data[7]);
        } else if (pkt->report_type == ESB_REPORT_CONSUMER) {
            LOG_DBG("RX CONS: %02X %02X", pkt->data[0], pkt->data[1]);
        }

        const int send_err = usb_hid_send(report_id, pkt->data, report_len);
        if (send_err) {
            LOG_WRN("usb_hid_send(id=%u) dropped: %d", report_id, send_err);
        }
        break;
    }

    case ESB_PKT_DISCONNECT:
        LOG_INF("DISCONNECT from keyboard, forgetting peer (-> UNPAIRED)");
        save_paired(false);
        m_state = STATE_UNPAIRED;
        led_status_set_armed(false);
#if IS_ENABLED(CONFIG_DONGLE_CHANNEL_HOP)
        channel_hop_dongle_set_paired(false);
#endif
        shell_relay_stop();
        break;

    case ESB_PKT_SHELL_DATA:
        if (len >= 2) {
            shell_relay_on_rx_data((const struct esb_pkt_shell_data *)data);
        }
        break;

    case ESB_PKT_SHELL_STOP:
        LOG_INF("SHELL_STOP rx, ending shell session");
        shell_relay_stop();
        if (IS_ENABLED(CONFIG_DONGLE_SHELL_RELAY_ALWAYS_ON) && m_state == STATE_PAIRED) {
            shell_relay_request();
        }
        break;

    case ESB_PKT_SHELL_POLL:
        shell_relay_on_keyboard_ack();
        break;

    case ESB_PKT_SHELL_BG_POLL:
        shell_relay_on_keyboard_bg_poll();
        break;

#if IS_ENABLED(CONFIG_DONGLE_CHANNEL_HOP)
    case ESB_PKT_CHANNEL_HOP_PROPOSAL:
        /* Only act on channel hop proposals from a verified peer — a
         * stranger keyboard must not be able to move our radio off the
         * pairing channel. */
        if (m_state == STATE_PAIRED) {
            channel_hop_dongle_on_rx_proposal(data, len);
        }
        break;
#endif

    case ESB_PKT_SHELL_START:
        /* Keyboard-triggered shell session (via &esb_shell_req behavior).
         * Mirrors the dongle pair-button short-press: spin up the CDC side,
         * queue SHELL_REQ in the next ACK so the keyboard flips into
         * active shell mode. Idempotent — shell_relay_request() no-ops if
         * already active. Gated on STATE_PAIRED so a stranger cannot coax
         * the dongle into enabling CDC I/O. */
        if (m_state == STATE_PAIRED) {
            LOG_INF("SHELL_START from keyboard, starting relay");
            shell_relay_request();
        }
        break;

    case ESB_PKT_VERIFY_REQ: {
        if (len < sizeof(struct esb_pkt_verify_req)) {
            break;
        }
        const struct esb_pkt_verify_req *req = (const void *)data;
        const bool sender_matches = (memcmp(req->device_id, m_peer_device_id,
                                            sizeof(m_peer_device_id)) == 0);

        if (m_state == STATE_VERIFYING && sender_matches) {
            struct esb_pkt_verify_resp resp = { .type = ESB_PKT_VERIFY_RESP };
            memcpy(resp.device_id, m_device_id, sizeof(resp.device_id));
            esb_prx_queue_ack(1, (uint8_t *)&resp, sizeof(resp));
            m_state = STATE_PAIRED;
            LOG_INF("VERIFY_REQ matched peer, session verified");
#if IS_ENABLED(CONFIG_DONGLE_CHANNEL_HOP)
            channel_hop_dongle_set_paired(true);
#endif
            if (IS_ENABLED(CONFIG_DONGLE_SHELL_RELAY_ALWAYS_ON)) {
                shell_relay_request();
            }
        } else if (m_state == STATE_PAIRED && sender_matches) {
            /* Already verified this boot; keyboard retrying while our
             * VERIFY_RESP is still in flight. Answer idempotently. */
            struct esb_pkt_verify_resp resp = { .type = ESB_PKT_VERIFY_RESP };
            memcpy(resp.device_id, m_device_id, sizeof(resp.device_id));
            esb_prx_queue_ack(1, (uint8_t *)&resp, sizeof(resp));
        } else {
            /* Unknown or non-matching sender. Covers:
             *   STATE_UNPAIRED           — settings were wiped
             *   STATE_{VERIFYING,PAIRED} — stranger keyboard with shared
             *                              ESB addresses
             * Nudge the sender to UNPAIRED so it stops spamming. */
            LOG_WRN("VERIFY_REQ from stranger %02X%02X%02X%02X%02X%02X, rejecting",
                    req->device_id[0], req->device_id[1], req->device_id[2],
                    req->device_id[3], req->device_id[4], req->device_id[5]);
            struct esb_pkt_disconnect d = { .type = ESB_PKT_DISCONNECT };
            esb_prx_queue_ack(1, (uint8_t *)&d, sizeof(d));
        }
        break;
    }

    case ESB_PKT_BENCH_PING:
        if (m_state == STATE_PAIRED) {
            bench_on_ping(esb_prx_get_last_rssi());
        }
        break;

    case ESB_PKT_BENCH_STOP:
        if (m_state == STATE_PAIRED) {
            bench_on_stop();
        }
        break;

    default:
        break;
    }

#if IS_ENABLED(CONFIG_DONGLE_CHANNEL_HOP)
    /* Post-dispatch hook: any inbound packet that didn't populate
     * committed_next is an opportunity to ride the next outbound ACK
     * with a REQUEST so the endpoint sends a fresh PROPOSAL on its
     * next TX. Runs after on_rx_proposal so a successful PROPOSAL is
     * not immediately followed by a redundant REQUEST. */
    channel_hop_dongle_after_rx(pipe);
#endif

    /* Periodic link-stats telemetry in the ACK stream. Opportunistic —
     * no-op unless the per-RX counter has crossed its interval. Last
     * in the RX flow so higher-priority ACK queueings (HOP_ACCEPT,
     * VERIFY_RESP, etc.) already sit ahead in the FIFO. */
    if (m_state == STATE_PAIRED) {
        esb_prx_maybe_queue_link_stats();
    }
}

int main(void) {
    int err;

    build_device_id();
    LOG_INF("dongle device_id: %02X%02X%02X%02X%02X%02X",
            m_device_id[0], m_device_id[1], m_device_id[2],
            m_device_id[3], m_device_id[4], m_device_id[5]);

    settings_subsys_init();
    settings_load_subtree("dongle");

    if (m_has_peer) {
        LOG_INF("Known peer %02X%02X%02X%02X%02X%02X, VERIFYING",
                m_peer_device_id[0], m_peer_device_id[1], m_peer_device_id[2],
                m_peer_device_id[3], m_peer_device_id[4], m_peer_device_id[5]);
        m_state = STATE_VERIFYING;
    } else {
        LOG_INF("No peer, UNPAIRED — press button to pair");
    }

    err = led_status_init();
    if (err) {
        LOG_ERR("LED status init: %d", err);
    }
    led_status_set_paired(m_has_peer);

    k_work_init(&btn_work, btn_work_fn);
    k_work_init_delayable(&long_press_work, long_press_work_fn);

    if (!device_is_ready(btn.port)) {
        LOG_ERR("GPIO not ready");
        return -ENODEV;
    }
    gpio_pin_configure_dt(&btn, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&btn_cb_data, btn_isr, BIT(btn.pin));
    gpio_add_callback(btn.port, &btn_cb_data);

    err = usb_hid_dongle_init();
    if (err) {
        LOG_ERR("USB HID init: %d", err);
        return err;
    }
    LOG_INF("USB HID init OK");

    err = esb_prx_init(on_esb_rx);
    if (err) {
        LOG_ERR("ESB PRX init: %d", err);
        return err;
    }
    LOG_INF("ESB PRX init OK on channel %u", CONFIG_DONGLE_ESB_RF_CHANNEL);

#if IS_ENABLED(CONFIG_DONGLE_CHANNEL_HOP)
    channel_hop_dongle_init(CONFIG_DONGLE_ESB_RF_CHANNEL);
    LOG_INF("channel hop enabled, initial channel %u", CONFIG_DONGLE_ESB_RF_CHANNEL);
#endif

    shell_relay_init();

    /* No boot-time shell_relay kick: STATE_VERIFYING must not emit anything
     * until the peer checks in. The kick happens on the VERIFYING → PAIRED
     * transition (fresh pair or matching VERIFY_REQ). */

    return 0;
}
