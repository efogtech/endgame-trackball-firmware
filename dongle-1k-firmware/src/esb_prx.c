/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <esb.h>
#include <zmk_esb/protocol.h>
#include "esb_prx.h"
#if IS_ENABLED(CONFIG_DONGLE_CHANNEL_HOP)
#include "channel_hop_dongle.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dongle_esb, LOG_LEVEL_INF);

/* Addresses matching the PTX */
static const uint8_t base_addr_0[4] = {0x17, 0xf4, 0x07, 0x7c};
static const uint8_t base_addr_1[4] = {0xb9, 0x8a, 0x16, 0x72};
static const uint8_t prefixes[2]    = {0x24, 0xc2};

static esb_prx_rx_cb_t m_rx_cb;
static struct esb_payload m_rx_payload;
static int8_t  m_last_rssi;
static uint8_t m_channel;

/* RSSI EWMA in dBm × 4 fixed-point so the ×3/4 IIR step keeps at least
 * two bits of resolution around typical operating points (-40..-85 dBm).
 * Initialised to 0 sentinel; first valid RX seeds directly. Update runs
 * in ISR context from the RX handler, so the single-word read in
 * esb_prx_get_rssi_ewma() is atomic on Cortex-M — no lock needed.
 * alpha = 1/4: ewma' = (ewma * 3 + sample * 4) / 4 (integer arithmetic
 * keeps it cheap — no divide). */
static int16_t m_rssi_ewma_q2;
static bool    m_rssi_ewma_valid;

/* Count of RX events since the last LINK_STATS ACK was queued. Incremented
 * in the RX handler, reset in esb_prx_maybe_queue_link_stats(). The actual
 * queueing runs on the RX dispatch thread (main.c) — ISR only counts. */
static uint32_t m_rx_since_link_stats;

/* RX dispatch runs on a dedicated cooperative thread drained from a message
 * queue filled by the ESB event handler (ISR). Each packet gets its own slot
 * so bursts don't overwrite each other. */
struct rx_msg {
    uint8_t pipe;
    uint8_t len;
    uint8_t data[ESB_MAX_PAYLOAD_LEN];
};

K_MSGQ_DEFINE(rx_msgq, sizeof(struct rx_msg), CONFIG_DONGLE_ESB_RX_MSGQ_DEPTH, 4);

K_THREAD_STACK_DEFINE(rx_stack, CONFIG_DONGLE_ESB_RX_STACK_SIZE);
static struct k_thread rx_thread;

static void rx_thread_fn(void *p1, void *p2, void *p3) {
    struct rx_msg msg;
    while (1) {
        k_msgq_get(&rx_msgq, &msg, K_FOREVER);
        if (m_rx_cb) {
            m_rx_cb(msg.pipe, msg.data, msg.len);
        }
    }
}

static void esb_evt_handler(struct esb_evt const *event) {
    switch (event->evt_id) {
    case ESB_EVENT_RX_RECEIVED:
        while (esb_read_rx_payload(&m_rx_payload) == 0) {
            if (m_rx_payload.length == 0 || m_rx_payload.length > ESB_MAX_PAYLOAD_LEN) {
                continue;
            }
            m_last_rssi = m_rx_payload.rssi;
            /* RSSI EWMA in Q2 (dBm × 4). Seed on first valid sample to
             * avoid a long converge from zero toward real values. */
            const int16_t sample_q2 = (int16_t)m_rx_payload.rssi * 4;
            if (!m_rssi_ewma_valid) {
                m_rssi_ewma_q2 = sample_q2;
                m_rssi_ewma_valid = true;
            } else {
                m_rssi_ewma_q2 =
                    (int16_t)(((int32_t)m_rssi_ewma_q2 * 3 + sample_q2) / 4);
            }
            m_rx_since_link_stats++;
            struct rx_msg msg = {
                .pipe = m_rx_payload.pipe,
                .len  = m_rx_payload.length,
            };
            memcpy(msg.data, m_rx_payload.data, m_rx_payload.length);
            if (k_msgq_put(&rx_msgq, &msg, K_NO_WAIT) != 0) {
                LOG_WRN("rx_msgq full, packet dropped");
            }
        }
        /* Watchdog rearm happens in the RX-dispatch thread (main.c) where
         * we can distinguish ESB_PKT_IDLE from everything else. The ISR
         * only stages packets. */
        break;
    case ESB_EVENT_TX_SUCCESS:
        break;
    case ESB_EVENT_TX_FAILED:
        LOG_WRN("ACK payload failed");
        break;
    }
}

int esb_prx_init(const esb_prx_rx_cb_t cb) {
    m_rx_cb = cb;

    k_thread_create(&rx_thread, rx_stack, K_THREAD_STACK_SIZEOF(rx_stack),
                    rx_thread_fn, NULL, NULL, NULL, K_PRIO_COOP(CONFIG_DONGLE_ESB_RX_THREAD_PRIORITY), 0, K_NO_WAIT);
    k_thread_name_set(&rx_thread, "esb_rx");

    struct esb_config cfg = ESB_DEFAULT_CONFIG;
    cfg.protocol        = ESB_PROTOCOL_ESB_DPL;
    cfg.mode            = ESB_MODE_PRX;
    cfg.bitrate         = ESB_BITRATE_1MBPS_BLE;
    cfg.crc             = ESB_CRC_16BIT;
    /* Must match the endpoint's use_fast_ramp_up — mismatched ramp
     * behavior between PTX and PRX causes deaf links. */
    cfg.use_fast_ramp_up = true;
    cfg.tx_output_power  = ESB_TX_POWER_8DBM;
    cfg.event_handler    = esb_evt_handler;
    cfg.payload_length   = ESB_MAX_PAYLOAD_LEN;

    int err = esb_init(&cfg);
    if (err) {
        LOG_ERR("esb_init: %d", err);
        return err;
    }

    esb_set_base_address_0(base_addr_0);
    esb_set_base_address_1(base_addr_1);
    esb_set_prefixes(prefixes, 2);
    m_channel = CONFIG_DONGLE_ESB_RF_CHANNEL;
    esb_set_rf_channel(m_channel);
    esb_enable_pipes(BIT(0) | BIT(1));

    err = esb_start_rx();
    if (err) {
        LOG_ERR("esb_start_rx: %d", err);
        return err;
    }

    return 0;
}

uint8_t esb_prx_get_channel(void) {
    return m_channel;
}

int esb_prx_set_channel(const uint8_t channel) {
    /* ESB docs: PRX must stop RX before changing channel, then restart. */
    int err = esb_stop_rx();
    if (err && err != -EINVAL) {
        /* -EINVAL here usually means "already stopped"; ignore. */
        LOG_WRN("esb_stop_rx: %d", err);
    }
    err = esb_set_rf_channel(channel);
    if (err) {
        LOG_ERR("esb_set_rf_channel(%u): %d", channel, err);
        /* Try to restart RX on the old channel anyway so we don't go deaf. */
        esb_start_rx();
        return err;
    }
    m_channel = channel;
    err = esb_start_rx();
    if (err) {
        LOG_ERR("esb_start_rx after hop: %d", err);
        return err;
    }
    return 0;
}

int esb_prx_queue_ack(const uint8_t pipe, const uint8_t *data, uint8_t len) {
    if (len > ESB_MAX_PAYLOAD_LEN) {
        return -EMSGSIZE;
    }
    struct esb_payload ack = {
        .pipe   = pipe,
        .length = len,
    };
    memcpy(ack.data, data, len);
    return esb_write_payload(&ack);
}

int esb_prx_flush_acks(void) {
    return esb_flush_tx();
}

int8_t esb_prx_get_last_rssi(void) {
    return m_last_rssi;
}

int8_t esb_prx_get_rssi_ewma(void) {
    if (!m_rssi_ewma_valid) {
        return 0;
    }
    /* Round-half-away-from-zero back from Q2 to dBm. RSSI is negative,
     * so biasing by -2 before integer-dividing matches rounding. */
    const int16_t v = m_rssi_ewma_q2;
    return (int8_t)((v >= 0 ? v + 2 : v - 2) / 4);
}

int esb_prx_maybe_queue_link_stats(void) {
    if (m_rx_since_link_stats < CONFIG_DONGLE_LINK_STATS_INTERVAL_RX) {
        return 0;
    }
    if (!m_rssi_ewma_valid) {
        /* No sample yet — nothing meaningful to report. */
        return 0;
    }
    const struct esb_pkt_link_stats s = {
        .type      = ESB_PKT_LINK_STATS,
        .rssi_last = m_last_rssi,
        .rssi_ewma = esb_prx_get_rssi_ewma(),
    };
    const int err = esb_prx_queue_ack(ESB_PIPE_DATA,
                                      (const uint8_t *)&s, sizeof(s));
    if (err == 0) {
        m_rx_since_link_stats = 0;
    }
    /* On queue-full (-EMSGSIZE / -ENOMEM from ESB lib), leave the counter
     * so the next RX re-attempts; opportunistic, no aggressive retry. */
    return err;
}
