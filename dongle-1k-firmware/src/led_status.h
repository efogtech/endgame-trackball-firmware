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

/* One-shot override: paint the status LED solid orange for
 * CONFIG_DONGLE_LED_HOP_FLASH_MS. Used to visually confirm that a channel
 * hop just fired. Safe to call from any context; the next hop flash
 * extends the deadline rather than stacking. */
void led_status_flash_hop(void);
