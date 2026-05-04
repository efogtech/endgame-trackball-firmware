/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 *
 * Dongle-side forwards USB CDC UART input to the keyboard over
 * ESB ACK payloads and writes keyboard shell output back to the CDC UART.
 *
 * A single button press when paired queues ESB_PKT_SHELL_REQ in an ACK
 * payload. The keyboard responds by entering shell relay mode and sending
 * periodic ESB_PKT_SHELL_POLL packets that drain any queued input.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

#include <zmk_esb/protocol.h>
#include "esb_prx.h"
#include "led_status.h"
#include "shell_relay.h"

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
LOG_MODULE_REGISTER(dongle_shell, LOG_LEVEL_INF);

#define CDC_DEV  DEVICE_DT_GET(DT_CHOSEN(zephyr_console))

RING_BUF_DECLARE(m_input_rb, CONFIG_DONGLE_SHELL_RELAY_INPUT_BUF_SIZE);
RING_BUF_DECLARE(m_output_rb, CONFIG_DONGLE_SHELL_RELAY_OUTPUT_BUF_SIZE);

static const struct device *m_cdc;
static volatile bool m_active;
static volatile bool m_keyboard_confirmed;
static bool m_echo;

static const char ECHO_OFF_CMD[] = "shell echo off";
#define ECHO_OFF_LEN (sizeof(ECHO_OFF_CMD) - 1)
static uint8_t m_echo_off_pos;

static void cdc_input_work_fn(struct k_work *w);
static void cdc_tx_work_fn(struct k_work *w);
static void shell_req_retry_work_fn(struct k_work *w);
static void inactivity_work_fn(struct k_work *w);
static void poll_watchdog_work_fn(struct k_work *w);
static void inactivity_reset(void);
static void poll_watchdog_kick(void);
static K_WORK_DEFINE(cdc_input_work, cdc_input_work_fn);
static K_WORK_DELAYABLE_DEFINE(cdc_tx_work, cdc_tx_work_fn);
static K_WORK_DELAYABLE_DEFINE(shell_req_retry_work, shell_req_retry_work_fn);
static K_WORK_DELAYABLE_DEFINE(inactivity_work, inactivity_work_fn);
static K_WORK_DELAYABLE_DEFINE(poll_watchdog_work, poll_watchdog_work_fn);

static void cdc_rx_isr(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);
    uart_irq_update(dev);
    if (!uart_irq_rx_ready(dev)) {
        return;
    }
    uint8_t buf[ESB_PKT_DATA_MAX];
    int n;
    while ((n = uart_fifo_read(dev, buf, sizeof(buf))) > 0) {
        if (!m_active) {
            continue;
        }
        ring_buf_put(&m_input_rb, buf, (uint32_t)n);
    }
    if (m_active) {
        k_work_submit(&cdc_input_work);
    }
}

static void cdc_tx_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (!m_active) {
        return;
    }
    uint8_t *chunk;
    uint32_t got;
    while ((got = ring_buf_get_claim(&m_output_rb, &chunk, CONFIG_DONGLE_SHELL_RELAY_TX_CHUNK_MAX)) > 0) {
        const int sent = uart_fifo_fill(m_cdc, chunk, (int)got);
        if (sent <= 0) {
            ring_buf_get_finish(&m_output_rb, 0);
            k_work_reschedule(&cdc_tx_work, K_MSEC(1));
            return;
        }
        ring_buf_get_finish(&m_output_rb, (uint32_t)sent);
        if ((uint32_t)sent < got) {
            k_work_reschedule(&cdc_tx_work, K_NO_WAIT);
            return;
        }
    }
}

static void cdc_tx_enqueue(const uint8_t *data, uint32_t len) {
    const uint32_t put = ring_buf_put(&m_output_rb, data, len);
    if (put < len) {
        LOG_WRN("TX buf full, dropped %u bytes", len - put);
    }
    k_work_reschedule(&cdc_tx_work, K_NO_WAIT);
}

static void echo_off_scan(const uint8_t *data, const uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        const char c = (char)data[i];
        if (c == '\r' || c == '\n') {
            if (m_echo_off_pos == ECHO_OFF_LEN) {
                m_echo = false;
            }
            m_echo_off_pos = 0;
        } else if (m_echo_off_pos == ECHO_OFF_LEN && (c == ' ' || c == '\t')) {
        } else if (m_echo_off_pos < ECHO_OFF_LEN && c == ECHO_OFF_CMD[m_echo_off_pos]) {
            m_echo_off_pos++;
        } else {
            m_echo_off_pos = (c == ECHO_OFF_CMD[0]) ? 1 : 0;
        }
    }
}

static void cdc_input_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (!m_active) {
        return;
    }
    if (!m_keyboard_confirmed) {
        ring_buf_reset(&m_input_rb);
        return;
    }
    uint8_t chunk[ESB_PKT_DATA_MAX];
    uint32_t got;
    uint32_t total = 0;
    while ((got = ring_buf_get(&m_input_rb, chunk, sizeof(chunk))) > 0) {
        if (m_echo) {
            cdc_tx_enqueue(chunk, got);
        }
        echo_off_scan(chunk, got);
        struct esb_pkt_shell_data pkt = {
            .type = ESB_PKT_SHELL_DATA,
            .len  = (uint8_t)got,
        };
        memcpy(pkt.data, chunk, got);
        const int err = esb_prx_queue_ack(1, (uint8_t *)&pkt, sizeof(pkt));
        if (err) {
            LOG_WRN("ACK queue err %d, dropped %u bytes", err, got);
        } else {
            total += got;
        }
    }
    if (total > 0) {
        LOG_DBG("queued %u cmd bytes as ACK payload(s)", total);
        inactivity_reset();
    }
}

void shell_relay_init(void) {
    m_cdc = CDC_DEV;
    if (!device_is_ready(m_cdc)) {
        LOG_ERR("CDC UART device not ready");
        return;
    }
    uart_irq_callback_user_data_set(m_cdc, cdc_rx_isr, NULL);
    LOG_INF("init OK (CDC %s)", m_cdc->name);
}

static void log_set_paused(const bool paused) {
    STRUCT_SECTION_FOREACH(log_backend, backend) {
        if (paused) {
            log_backend_disable(backend);
        } else {
            log_backend_enable(backend, backend->cb->ctx, LOG_LEVEL_DBG);
        }
    }
}

static void inactivity_reset(void) {
    if (CONFIG_DONGLE_SHELL_RELAY_INACTIVITY_S > 0) {
        k_work_reschedule(&inactivity_work, K_SECONDS(CONFIG_DONGLE_SHELL_RELAY_INACTIVITY_S));
    }
}

static void poll_watchdog_kick(void) {
    if (CONFIG_DONGLE_SHELL_RELAY_POLL_WATCHDOG_MS > 0) {
        k_work_reschedule(&poll_watchdog_work, K_MSEC(CONFIG_DONGLE_SHELL_RELAY_POLL_WATCHDOG_MS));
    }
}

/* Fires when no SHELL_POLL has arrived for the watchdog window. Typical cause:
 * the keyboard switched endpoints (ESB → BLE) so the PTX is gone and our ACK
 * FIFO is filling with undeliverable payloads. Flush the stale queue, demote
 * to unconfirmed, and re-queue SHELL_REQ. The watchdog self-rearms so we keep
 * retrying periodically — a single re-request can get buried in a FIFO full
 * of stale SHELL_REQs from earlier retries, so each window we start fresh.
 *
 * m_output_rb is NOT reset here: it carries keyboard→CDC bytes that may have
 * arrived just before the watchdog fired (a long-running keyboard command
 * blocks SHELL_POLL but still bursts SHELL_DATA on completion). Wiping it
 * silently drops the user's command output. */
static void poll_watchdog_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (!m_active) {
        return;
    }
    if (m_keyboard_confirmed) {
        LOG_WRN("no SHELL_POLL for %dms, re-requesting session",
                CONFIG_DONGLE_SHELL_RELAY_POLL_WATCHDOG_MS);
        m_keyboard_confirmed = false;
        m_echo = true;
        m_echo_off_pos = 0;
        ring_buf_reset(&m_input_rb);
        led_status_set_shell_relay(false);
        led_status_set_shell_pending(true);
    } else {
        LOG_DBG("watchdog retry, no poll yet");
    }

    esb_prx_flush_acks();
    struct esb_pkt_shell_req req = { .type = ESB_PKT_SHELL_REQ };
    const int err = esb_prx_queue_ack(1, (uint8_t *)&req, sizeof(req));
    if (err && err != -ENOMEM) {
        LOG_WRN("re-queue SHELL_REQ failed: %d", err);
    }
    k_work_reschedule(&shell_req_retry_work, K_MSEC(CONFIG_DONGLE_SHELL_RELAY_RETRY_MS));
    poll_watchdog_kick();
}

static void inactivity_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (IS_ENABLED(CONFIG_DONGLE_SHELL_RELAY_ALWAYS_ON)) {
        LOG_INF("inactivity timeout, re-requesting (always-on)");
        shell_relay_stop();
        shell_relay_request();
        return;
    }
    LOG_INF("inactivity timeout, stopping");
    shell_relay_stop();
}

static void shell_req_retry_work_fn(struct k_work *w) {
    if (!m_active || m_keyboard_confirmed) {
        return;
    }
    struct esb_pkt_shell_req req = { .type = ESB_PKT_SHELL_REQ };
    const int err = esb_prx_queue_ack(1, (uint8_t *)&req, sizeof(req));
    if (err == -ENOMEM) {
        LOG_DBG("SHELL_REQ FIFO full, waiting for keyboard TX");
        k_work_reschedule(&shell_req_retry_work, K_MSEC(CONFIG_DONGLE_SHELL_RELAY_FIFO_FULL_RETRY_MS));
    } else {
        if (err) {
            LOG_WRN("retry SHELL_REQ failed: %d", err);
        } else {
            LOG_DBG("retried SHELL_REQ ACK");
        }
        k_work_reschedule(&shell_req_retry_work, K_MSEC(CONFIG_DONGLE_SHELL_RELAY_RETRY_MS));
    }
}

void shell_relay_request(void) {
    if (m_active) {
        LOG_DBG("request ignored (already active)");
        return;
    }
    LOG_INF("shell relay: requesting session, queuing SHELL_REQ ACK");
    m_active = true;
    m_keyboard_confirmed = false;
    m_echo = true;
    m_echo_off_pos = 0;
    led_status_set_shell_relay(false);
    led_status_set_shell_pending(true);
    ring_buf_reset(&m_input_rb);
    ring_buf_reset(&m_output_rb);

    struct esb_pkt_shell_req req = { .type = ESB_PKT_SHELL_REQ };
    const int err = esb_prx_queue_ack(1, (uint8_t *)&req, sizeof(req));
    if (err) {
        LOG_ERR("failed to queue SHELL_REQ ACK: %d", err);
    } else {
        LOG_DBG("SHELL_REQ ACK queued on pipe 1");
    }

    k_work_reschedule(&shell_req_retry_work, K_MSEC(CONFIG_DONGLE_SHELL_RELAY_RETRY_MS));
    inactivity_reset();
    poll_watchdog_kick();
    uart_irq_rx_enable(m_cdc);
    log_set_paused(true);
}

void shell_relay_on_keyboard_ack(void) {
    if (!m_active) {
        return;
    }
    inactivity_reset();
    poll_watchdog_kick();
    if (m_keyboard_confirmed) {
        return;
    }
    m_keyboard_confirmed = true;
    led_status_set_shell_pending(false);
    led_status_set_shell_relay(true);
    k_work_cancel_delayable(&shell_req_retry_work);
    LOG_INF("shell relay: keyboard confirmed (SHELL_POLL rx)");
}

/* Keyboard is paired but NOT in shell mode — its SHELL_REQ either got lost or
 * m_shell_active was already torn down. Demote to pending (keyboard_confirmed
 * = false), make sure shell_req_retry_work is running, and nudge one fresh
 * SHELL_REQ immediately so the next bg_poll ACK can carry it. No flushing:
 * the queue may contain user-typed SHELL_DATA from before the demote. */
void shell_relay_on_keyboard_bg_poll(void) {
    if (!m_active) {
        return;
    }
    inactivity_reset();
    poll_watchdog_kick();

    if (m_keyboard_confirmed) {
        m_keyboard_confirmed = false;
        led_status_set_shell_relay(false);
        led_status_set_shell_pending(true);
        LOG_INF("shell relay: keyboard dropped to bg_poll, demoting to pending");
    }

    struct esb_pkt_shell_req req = { .type = ESB_PKT_SHELL_REQ };
    const int err = esb_prx_queue_ack(1, (uint8_t *)&req, sizeof(req));
    if (err && err != -ENOMEM) {
        LOG_WRN("BG_POLL SHELL_REQ nudge failed: %d", err);
    }
    k_work_reschedule(&shell_req_retry_work, K_MSEC(CONFIG_DONGLE_SHELL_RELAY_RETRY_MS));
}

void shell_relay_stop(void) {
    if (!m_active) {
        return;
    }
    k_work_cancel_delayable(&cdc_tx_work);
    ring_buf_reset(&m_output_rb);
    log_set_paused(false);
    LOG_INF("shell relay: stopped");
    m_active = false;
    m_keyboard_confirmed = false;
    m_echo = false;
    m_echo_off_pos = 0;
    k_work_cancel_delayable(&shell_req_retry_work);
    k_work_cancel_delayable(&inactivity_work);
    k_work_cancel_delayable(&poll_watchdog_work);
    led_status_set_shell_relay(false);
    led_status_set_shell_pending(false);
    uart_irq_rx_disable(m_cdc);
    ring_buf_reset(&m_input_rb);
}

void shell_relay_on_rx_data(const struct esb_pkt_shell_data *pkt) {
    if (!m_active || pkt->len == 0 || pkt->len > ESB_PKT_DATA_MAX) {
        LOG_DBG("SHELL_DATA ignored (active=%d len=%u)", m_active, pkt->len);
        return;
    }
    inactivity_reset();
    poll_watchdog_kick();
    LOG_DBG("output %u bytes → CDC UART", pkt->len);
    cdc_tx_enqueue(pkt->data, pkt->len);
}

bool shell_relay_is_active(void) {
    return m_active;
}
