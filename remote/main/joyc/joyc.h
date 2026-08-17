/*
 * JoyC (M5 Hat Mini JoyC, I2C addr 0x54) reader, isolated in its own C TU.
 *
 * WHY A SEPARATE FILE: the espressif/i2c_bus component (with
 * CONFIG_I2C_BUS_BACKWARD_CONFIG unset, i.e. the factory setting) pulls in the
 * NEW driver/i2c_master.h AND defines its own i2c_config_t. M5Unified.h pulls in
 * the legacy driver/i2c.h which ALSO defines i2c_config_t. Mixing both in one
 * translation unit is a hard "conflicting declaration" compile error. The
 * factory firmware sidesteps this by keeping all i2c_bus usage in a pure-C file
 * (joystick_handle.c) that never includes M5Unified. We do the same here.
 *
 * At runtime this means the JoyC rides the NEW i2c_master driver (via i2c_bus),
 * exactly like the OOTB firmware, coexisting with m5gfx's internal I2C on a
 * different port. Forcing everything to the legacy driver (what the previous run
 * did with BACKWARD_CONFIG=y + a m5gfx patch) is what broke reads (ok=0).
 */
#ifndef JOYC_H
#define JOYC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Create the I2C bus on I2C_NUM_0 (HAT pins G0=SDA, G26=SCL) and the JoyC
 * device at 0x54, then validate with two reads. Returns true if the first read
 * succeeded. Must be called AFTER M5.begin()+M5.Imu.init() so the internal I2C
 * (In_I2C = I2C_NUM_1) is already claimed and NUM_0 is free. */
bool joyc_init(void);

/* Read raw 16-bit stick X/Y and click (true = pressed, reg 0x30 active-low).
 * Returns false on I2C error. */
bool joyc_read(uint16_t *x, uint16_t *y, bool *click);

/* Last-known health of the JoyC link. */
bool joyc_ok(void);

#ifdef __cplusplus
}
#endif

#endif /* JOYC_H */
