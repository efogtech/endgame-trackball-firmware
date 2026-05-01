/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

int usb_hid_dongle_init(void);
int usb_hid_send(uint8_t report_type, const uint8_t *data, uint8_t len);
