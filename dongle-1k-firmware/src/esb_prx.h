/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef void (*esb_prx_rx_cb_t)(uint8_t pipe, const uint8_t *data, uint8_t len);

int    esb_prx_init(esb_prx_rx_cb_t cb);
int    esb_prx_queue_ack(uint8_t pipe, const uint8_t *data, uint8_t len);
int    esb_prx_flush_acks(void);
int8_t esb_prx_get_last_rssi(void);

/* EWMA of RX RSSI in dBm. Returns 0 before any RX has been observed. */
int8_t esb_prx_get_rssi_ewma(void);

/* Queue an ESB_PKT_LINK_STATS ACK payload if enough RXes have accumulated
 * since the last queueing (CONFIG_DONGLE_LINK_STATS_INTERVAL_RX). Called
 * from the RX dispatch thread after every received packet — cheap no-op
 * when the interval has not elapsed. Returns 0 on successful queue or
 * skip; negative errno if the ACK queue rejected the payload. */
int    esb_prx_maybe_queue_link_stats(void);

/* Current RF channel. */
uint8_t esb_prx_get_channel(void);

/* Change the active RF channel: stop RX, apply channel, restart RX.
 * Must not be called from ISR context. */
int esb_prx_set_channel(uint8_t channel);
