/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Thin C-callable shim over M5Unified's C++ API so the C power/LED code can
 * share M5's SINGLE internal-I2C owner (M5.In_I2C) instead of standing up a
 * SECOND ESP-IDF i2c_master bus on I2C_NUM_0.
 *
 * WHY THIS EXISTS (root cause of the "reply never plays" regression):
 * The new ESP-IDF i2c_master driver allows exactly ONE bus owner per port.
 * M5Unified owns I2C_NUM_0 through its own lgfx stack (M5.In_I2C) and drives
 * EVERYTHING on the CoreS3 internal bus through it: AXP2101 PMIC, AW88298
 * speaker codec, ES7210 mic codec, AW9523 expander, PY32 LED expander. Our
 * power-button code used to call i2c_new_master_bus(I2C_NUM_0) at boot to talk
 * to the AXP2101 raw -> a SECOND owner. Once that existed, M5Unified's
 * Speaker.begin() (AW88298 codec config) failed on the contended bus and the
 * TTS reply never played. Routing our AXP + LED access through M5.In_I2C /
 * M5.Power removes the second owner: everything serializes at lgfx's own i2c
 * lock, so the two-owner race is gone structurally.
 *
 * All I2C helpers here return true on success, false on failure. A failed read
 * of the AXP power-key register returns 0 ("no key") via M5's value_or(0), so a
 * contended read can never be misread as a phantom press.
 */

/* AXP2101 power-key state via M5.Power.getKeyState():
 *   0 = none, 1 = long press, 2 = short click, 3 = both.
 * Reads AXP reg 0x49 (mask 0x0C) through M5.In_I2C and auto-clears (W1C). */
uint8_t stackchan_m5_get_key_state(void);

/* Soft power-off through M5.Power.powerOff() (cuts all rails; same soft
 * power-off the old raw AXP2101 reg 0x10 bit0 did, but through the shared
 * owner). Does not return on success (the chip powers down). */
void stackchan_m5_power_off(void);

/* Poll the CoreS3 capacitive touchscreen; returns true once per fresh tap
 * (rising edge). Reads FT6336 through lgfx's touch driver, which shares the
 * lgfx i2c lock with M5.In_I2C -- safe to call from the power-key poll loop. */
bool stackchan_m5_touch_tapped(void);

/* Internal-I2C (M5.In_I2C) register access, addressed by 7-bit device address.
 * These replace the raw i2c_master dev_* helpers; the register sequences are
 * unchanged, only the transport moved onto the shared owner. */
bool stackchan_m5_i2c_read8(uint8_t addr, uint8_t reg, uint8_t *out, uint32_t freq);
bool stackchan_m5_i2c_write8(uint8_t addr, uint8_t reg, uint8_t val, uint32_t freq);
bool stackchan_m5_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data,
                            size_t len, uint32_t freq);
bool stackchan_m5_i2c_bit_on(uint8_t addr, uint8_t reg, uint8_t mask, uint32_t freq);

#ifdef __cplusplus
}
#endif
