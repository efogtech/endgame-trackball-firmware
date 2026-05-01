/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <zmk_esb/protocol.h>

void shell_relay_init(void);
void shell_relay_request(void);
void shell_relay_stop(void);
void shell_relay_on_rx_data(const struct esb_pkt_shell_data *pkt);
void shell_relay_on_keyboard_ack(void);     /* keyboard SHELL_POLL: active shell confirmed */
void shell_relay_on_keyboard_bg_poll(void); /* keyboard SHELL_BG_POLL: paired but not in shell */
bool shell_relay_is_active(void);
