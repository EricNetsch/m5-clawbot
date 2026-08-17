/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief One-shot boot bring-up for the StackChan body.
 *
 * Runs M5Unified's board init (power management + display), draws a boot
 * splash on the LCD, and lights the 12 RGB status LEDs so the device visibly
 * shows it is powered on. Safe to call once, early in app_main.
 */
void stackchan_boot(void);

#ifdef __cplusplus
}
#endif
