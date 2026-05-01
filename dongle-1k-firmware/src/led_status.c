/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 *
 * WS2812 status LED. Per-state RGB and timing come from CONFIG_DONGLE_LED_*
 * — the labels below describe role, not literal colour:
 *   !paired, !armed       -> CONFIG_DONGLE_LED_UNPAIRED_*
 *   !paired, armed        -> blink CONFIG_DONGLE_LED_ARMED_* at
 *                            CONFIG_DONGLE_LED_BLINK_HALF_PERIOD_MS
 *   paired, no recent RX  -> CONFIG_DONGLE_LED_IDLE_*
 *   paired, recent RX     -> CONFIG_DONGLE_LED_CONNECTED_*
 *   paired                -> SPI bus suspended, LED dark
 *                            (when CONFIG_DONGLE_NO_ACTIVE_EFFECTS=y)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/atomic.h>
#ifndef CONFIG_DONGLE_NOLED
#include <zephyr/drivers/led_strip.h>
#if IS_ENABLED(CONFIG_DONGLE_NO_ACTIVE_EFFECTS)
#include <zephyr/pm/device.h>
#endif
#endif

#include "led_status.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dongle_led, LOG_LEVEL_INF);

#ifdef CONFIG_DONGLE_NOLED

int led_status_init(void) { return 0; }
void led_status_set_paired(const bool paired) { ARG_UNUSED(paired); }
void led_status_set_armed(const bool armed) { ARG_UNUSED(armed); }
void led_status_set_shell_relay(const bool active) { ARG_UNUSED(active); }
void led_status_set_shell_pending(const bool pending) { ARG_UNUSED(pending); }
void led_status_mark_rx(void) {}
void led_status_flash_hop(void) {}

#else

#define TICK_MS                 CONFIG_DONGLE_LED_TICK_MS
#define BLINK_HALF_PERIOD_MS    CONFIG_DONGLE_LED_BLINK_HALF_PERIOD_MS
#define CONNECTED_TIMEOUT_MS    CONFIG_DONGLE_LED_CONNECTED_TIMEOUT_MS

static const struct device *strip = DEVICE_DT_GET(DT_CHOSEN(zephyr_led_strip));
#if IS_ENABLED(CONFIG_DONGLE_NO_ACTIVE_EFFECTS)
static const struct device *spi_bus = DEVICE_DT_GET(DT_BUS(DT_CHOSEN(zephyr_led_strip)));
static bool m_spi_suspended;
#endif

static atomic_t m_paired;
static atomic_t m_armed;
static atomic_t m_shell_relay;
static atomic_t m_shell_pending;
static atomic_t m_last_rx_ms;
static atomic_t m_hop_flash_until_ms;

static uint8_t scale(const uint8_t v) {
    return (uint8_t)(((uint32_t)v * CONFIG_DONGLE_LED_BRIGHTNESS) / 100U);
}

static void set_rgb(const uint8_t r, const uint8_t g, const uint8_t b) {
    struct led_rgb px = {
        .r = scale(r),
        .g = scale(g),
        .b = scale(b),
    };
    led_strip_update_rgb(strip, &px, 1);
}

#if IS_ENABLED(CONFIG_DONGLE_NO_ACTIVE_EFFECTS)
static void resume_spi_if_needed(void) {
    if (!m_spi_suspended) {
        return;
    }
    const int err = pm_device_action_run(spi_bus, PM_DEVICE_ACTION_RESUME);
    if (err && err != -EALREADY) {
        LOG_WRN("SPI resume failed: %d", err);
    }
    m_spi_suspended = false;
}

static void suspend_spi_if_needed(void) {
    if (m_spi_suspended) {
        return;
    }
    /* Drive zeros once so the WS2812 latches dark before the bus goes
     * away — the LED holds whatever was last clocked in. */
    const struct led_rgb px = { 0 };
    led_strip_update_rgb(strip, &px, 1);
    const int err = pm_device_action_run(spi_bus, PM_DEVICE_ACTION_SUSPEND);
    if (err && err != -EALREADY) {
        LOG_WRN("SPI suspend failed: %d", err);
        return;
    }
    m_spi_suspended = true;
}
#endif

static void led_thread_fn(void *p1, void *p2, void *p3) {
    uint32_t phase = 0;
    while (1) {
#if IS_ENABLED(CONFIG_DONGLE_NO_ACTIVE_EFFECTS)
        if (atomic_get(&m_paired)) {
            suspend_spi_if_needed();
            phase++;
            k_sleep(K_MSEC(TICK_MS));
            continue;
        }
        resume_spi_if_needed();
#endif

        const uint32_t now_ms = (uint32_t)k_uptime_get();
        const uint32_t hop_until = (uint32_t)atomic_get(&m_hop_flash_until_ms);
        if (hop_until != 0 && (int32_t)(hop_until - now_ms) > 0) {
            set_rgb(CONFIG_DONGLE_LED_HOP_R,
                    CONFIG_DONGLE_LED_HOP_G,
                    CONFIG_DONGLE_LED_HOP_B);
            phase++;
            k_sleep(K_MSEC(TICK_MS));
            continue;
        }

        const bool paired = atomic_get(&m_paired);

        if (!paired) {
            const bool armed = atomic_get(&m_armed);
            if (!armed) {
                set_rgb(CONFIG_DONGLE_LED_UNPAIRED_R, CONFIG_DONGLE_LED_UNPAIRED_G, CONFIG_DONGLE_LED_UNPAIRED_B);
            } else {
                const bool on = ((phase * TICK_MS) / BLINK_HALF_PERIOD_MS) & 1U;
                set_rgb(on ? CONFIG_DONGLE_LED_ARMED_R : 0,
                        on ? CONFIG_DONGLE_LED_ARMED_G : 0,
                        on ? CONFIG_DONGLE_LED_ARMED_B : 0);
            }
        } else {
            const bool shell = atomic_get(&m_shell_relay);
            const bool shell_pending = atomic_get(&m_shell_pending);
            if (shell) {
                set_rgb(CONFIG_DONGLE_LED_SHELL_RELAY_R, CONFIG_DONGLE_LED_SHELL_RELAY_G, CONFIG_DONGLE_LED_SHELL_RELAY_B);
            } else if (shell_pending) {
                set_rgb(CONFIG_DONGLE_LED_SHELL_PENDING_R, CONFIG_DONGLE_LED_SHELL_PENDING_G, CONFIG_DONGLE_LED_SHELL_PENDING_B);
            } else {
                const uint32_t last = (uint32_t)atomic_get(&m_last_rx_ms);
                const bool connected = (uint32_t)(now_ms - last) < CONNECTED_TIMEOUT_MS;
                if (connected) {
                    set_rgb(CONFIG_DONGLE_LED_CONNECTED_R, CONFIG_DONGLE_LED_CONNECTED_G, CONFIG_DONGLE_LED_CONNECTED_B);
                } else {
                    set_rgb(CONFIG_DONGLE_LED_IDLE_R, CONFIG_DONGLE_LED_IDLE_G, CONFIG_DONGLE_LED_IDLE_B);
                }
            }
        }

        phase++;
        k_sleep(K_MSEC(TICK_MS));
    }
}

K_THREAD_STACK_DEFINE(led_stack, CONFIG_DONGLE_LED_STACK_SIZE);
static struct k_thread led_thread;

int led_status_init(void) {
    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }

    k_thread_create(&led_thread, led_stack, K_THREAD_STACK_SIZEOF(led_stack), led_thread_fn,
        NULL, NULL, NULL, K_PRIO_PREEMPT(CONFIG_DONGLE_LED_THREAD_PRIORITY), 0, K_NO_WAIT);
    k_thread_name_set(&led_thread, "led_status");
    LOG_INF("LED status thread started");
    return 0;
}

void led_status_set_paired(const bool paired) {
    atomic_set(&m_paired, paired ? 1 : 0);
}

void led_status_set_armed(const bool armed) {
    atomic_set(&m_armed, armed ? 1 : 0);
}

void led_status_set_shell_relay(const bool active) {
    atomic_set(&m_shell_relay, active ? 1 : 0);
}

void led_status_set_shell_pending(const bool pending) {
    atomic_set(&m_shell_pending, pending ? 1 : 0);
}

void led_status_mark_rx(void) {
    atomic_set(&m_last_rx_ms, (atomic_val_t)(uint32_t)k_uptime_get());
}

void led_status_flash_hop(void) {
    const uint32_t now = (uint32_t)k_uptime_get();
    const uint32_t deadline = now + (uint32_t)CONFIG_DONGLE_LED_HOP_FLASH_MS;
    atomic_set(&m_hop_flash_until_ms, (atomic_val_t)deadline);
}

#endif /* CONFIG_DONGLE_NOLED */
