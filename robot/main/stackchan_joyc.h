/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Probe for an M5 Mini JoyC (U156) joystick unit on the CoreS3 external
 *        I2C bus (Port A, red Grove -> M5.Ex_I2C). If present, spawn a task that
 *        reads the stick and drives the head servos live via the existing
 *        stackchan_servo API. If not present, log a warning and return ESP_OK
 *        (firmware keeps running; the idle look-around stays active).
 */
esp_err_t stackchan_joyc_start(void);

#ifdef __cplusplus
}
#endif
