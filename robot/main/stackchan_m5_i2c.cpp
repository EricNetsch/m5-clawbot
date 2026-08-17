/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * C-callable shim over M5Unified (M5.Power / M5.In_I2C). See stackchan_m5_i2c.h
 * for the root-cause rationale (single internal-I2C owner => no second bus).
 */

#include "stackchan_m5_i2c.h"

#include <M5Unified.h>

extern "C" uint8_t stackchan_m5_get_key_state(void)
{
    return M5.Power.getKeyState();
}

extern "C" void stackchan_m5_power_off(void)
{
    M5.Power.powerOff();
}

/* Poll the CoreS3 capacitive touchscreen for a fresh tap (rising edge).
 * Reads the FT6336 through lgfx's touch driver, which shares the same lgfx
 * i2c lock that serializes M5.In_I2C -- so it coexists with the power-key
 * poll and audio codec on the single internal bus. Call at a steady cadence
 * (the power_button_task's 40ms loop); returns true exactly once per new
 * press. */
extern "C" bool stackchan_m5_touch_tapped(void)
{
    if (!M5.Touch.isEnabled()) { return false; }
    M5.Touch.update(m5gfx::millis());
    return M5.Touch.getDetail().wasPressed();
}

extern "C" bool stackchan_m5_i2c_read8(uint8_t addr, uint8_t reg, uint8_t *out,
                                       uint32_t freq)
{
    if (out == nullptr) { return false; }
    return M5.In_I2C.readRegister(addr, reg, out, 1, freq);
}

extern "C" bool stackchan_m5_i2c_write8(uint8_t addr, uint8_t reg, uint8_t val,
                                        uint32_t freq)
{
    return M5.In_I2C.writeRegister8(addr, reg, val, freq);
}

extern "C" bool stackchan_m5_i2c_write(uint8_t addr, uint8_t reg,
                                       const uint8_t *data, size_t len,
                                       uint32_t freq)
{
    return M5.In_I2C.writeRegister(addr, reg, data, len, freq);
}

extern "C" bool stackchan_m5_i2c_bit_on(uint8_t addr, uint8_t reg, uint8_t mask,
                                        uint32_t freq)
{
    return M5.In_I2C.bitOn(addr, reg, mask, freq);
}
