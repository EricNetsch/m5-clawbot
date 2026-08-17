/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ambient-light auto-brightness for the CoreS3's onboard LTR-553ALS.
 *
 * The LTR-553ALS sits at 7-bit I2C addr 0x23 on the CoreS3 INTERNAL bus, so it
 * is reached through the shared M5.In_I2C owner (via stackchan_m5_i2c_*), NOT a
 * second i2c_master bus (see stackchan_m5_i2c.h for why that matters). We read
 * the visible ALS channel, map it to an LCD backlight level with a comfortable
 * floor/ceiling, and apply it: once at startup and then every ~60s from a small
 * background task, smoothing between samples so the screen never steps harshly.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Activate the LTR-553ALS (ALS active mode) and, once it has a first sample,
 * set the LCD backlight to match ambient light. Then start a background task
 * that re-samples every ~60s and eases the backlight toward the new target.
 * Safe to call once, after stackchan_boot() (which owns M5/display). */
esp_err_t stackchan_light_start(void);

/* Read the visible ALS channel (raw LTR-553 Ch0 counts). Returns true and sets
 * *out_raw on success; false if the sensor isn't ready / read failed. */
bool stackchan_light_read_raw(uint16_t *out_raw);

/* Map a raw ALS reading to a backlight level (0-255) using the floor/ceiling
 * curve. Exposed for logging/tuning. */
uint8_t stackchan_light_raw_to_brightness(uint16_t raw);

#ifdef __cplusplus
}
#endif
