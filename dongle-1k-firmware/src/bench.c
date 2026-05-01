/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zmk_esb/protocol.h>
#include "bench.h"
#include "esb_prx.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dongle_bench, LOG_LEVEL_INF);

static int32_t m_rssi_sum;
static int8_t  m_rssi_min;
static int8_t  m_rssi_max;
static uint32_t m_rx_count;
static bool m_active;

/* Saved result so repeated BENCH_STOP re-queues the same ACK payload.
 * The keyboard keeps sending pings while STOPPING; the dongle needs a
 * stable result to put in each ACK until the keyboard sees it. */
static struct esb_pkt_bench_result m_saved_result;
static bool m_result_ready;

void bench_on_ping(const int8_t rssi) {
    if (!m_active) {
        /* First ping of a new run — reset everything. */
        LOG_INF("Bench: run started (first ping rssi=%d)", rssi);
        m_active       = true;
        m_result_ready = false;
        m_rssi_sum     = rssi;
        m_rx_count     = 1;
        m_rssi_min     = rssi;
        m_rssi_max     = rssi;
        return;
    }
    m_rssi_sum += rssi;
    m_rx_count++;
    if (rssi < m_rssi_min) {
        m_rssi_min = rssi;
    }
    if (rssi > m_rssi_max) {
        m_rssi_max = rssi;
    }
}

void bench_on_stop(void) {
    if (!m_result_ready) {
        m_saved_result.type = ESB_PKT_BENCH_RESULT;
        if (m_active && m_rx_count > 0) {
            m_saved_result.rssi_avg = (int8_t)(m_rssi_sum / (int32_t)m_rx_count);
            m_saved_result.rssi_min = m_rssi_min;
            m_saved_result.rssi_max = m_rssi_max;
            m_saved_result.rx_count = m_rx_count;
            LOG_INF("Bench: %u pings, RSSI avg=%d min=%d max=%d",
                    m_rx_count, (int)m_saved_result.rssi_avg,
                    (int)m_saved_result.rssi_min, (int)m_saved_result.rssi_max);
        } else {
            m_saved_result.rssi_avg = 0;
            m_saved_result.rssi_min = 0;
            m_saved_result.rssi_max = 0;
            m_saved_result.rx_count = 0;
            LOG_WRN("Bench: BENCH_STOP received but no pings accumulated");
        }
        m_result_ready = true;
        m_active       = false;
    }

    /* Re-queue on every BENCH_STOP so the keyboard's next ping carries it. */
    esb_prx_queue_ack(1, (uint8_t *)&m_saved_result, sizeof(m_saved_result));
}
