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
 * @brief Start the ESP-NOW receiver that lets the M5 StackChan battery joystick
 *        remote drive the head servos, matching the M5 factory-firmware
 *        "ESP-NOW wireless remote control" demo protocol.
 *
 * Must be called after Wi-Fi has been started (the node brings Wi-Fi up as a
 * station). Internally this spawns a task that waits for Wi-Fi, calls
 * esp_now_init() on top of the existing station interface (it does NOT touch
 * Wi-Fi mode / channel / connection), registers a recv callback, and maps
 * incoming joystick packets to stackchan_servo_move().
 */
esp_err_t stackchan_espnow_remote_start(void);

#ifdef __cplusplus
}
#endif
