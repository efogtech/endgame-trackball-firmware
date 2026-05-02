/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>

int  led_status_init(void);
void led_status_set_paired(bool paired);
void led_status_set_armed(bool armed);
void led_status_set_shell_relay(bool active);
void led_status_set_shell_pending(bool pending);
void led_status_mark_rx(void);

/* One-shot override: paint the status LED with CONFIG_DONGLE_LED_NOLINK_*
 * for CONFIG_DONGLE_LED_NOLINK_FLASH_MS. Fired on every channel hop
 * (speculative or cooperative) — both are symptoms of link degradation,
 * so they share a single visual cue. Safe to call from any context; a
 * later flash extends the deadline rather than stacking. */
void led_status_flash_nolink(void);

/* Solid override: while true, paint the status LED with
 * CONFIG_DONGLE_LED_NOLINK_* whenever the dongle is paired and not in a
 * shell relay state. Asserted by the channel-hop machinery when the peer
 * cannot be reached (rollback dwell, post-validate-fail revert window),
 * cleared on the next confirmed RX. */
void led_status_set_link_lost(bool lost);
