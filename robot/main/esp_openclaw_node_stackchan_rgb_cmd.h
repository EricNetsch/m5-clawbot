/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"
#include "esp_openclaw_node.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Cooperative pause counter for the shared internal I2C bus.
 *
 * Retained for the LED transport (stackchan_boot.cpp: stackchan_led_fill) and
 * the voice codec BUS_LOCK, which still bump it around their M5.In_I2C bursts.
 *
 * The power-key poller NO LONGER reads it: the raw second i2c_master bus on
 * I2C_NUM_0 is gone. Everything on the CoreS3 internal bus (AXP2101 PMIC via
 * M5.Power, AW88298/ES7210 codecs, AW9523/PY32 expanders) now goes through
 * M5.In_I2C -- the SINGLE bus owner -- so it all serializes at lgfx's own i2c
 * lock. The two-owner race that made M5.Speaker.begin() fail (reply never
 * played) and misread the AXP as a phantom power-key press is gone
 * structurally, so this counter is now purely advisory for the LED/codec call
 * sites and left in place so they compile and behave unchanged.
 */
extern volatile int g_stackchan_power_poll_pause;

#ifdef __cplusplus
}
#endif

/**
 * @brief Register the StackChan RGB LED commands.
 *
 * StackChan's 12x WS2812C LEDs are NOT driven directly from an ESP32 GPIO.
 * They hang off a PY32L020 I2C IO-expander (address 0x6F) on the CoreS3
 * internal I2C bus (SDA = GPIO12, SCL = GPIO11). The expander chip has
 * built-in WS2812 driving logic; the ESP32 only ever writes colors into the
 * expander's LED RAM over I2C and then triggers a latch/refresh.
 *
 * Commands added:
 * - `rgb.set`     { index:0-11, r:0-255, g:0-255, b:0-255 } — buffer one LED
 * - `rgb.fill`    { r:0-255, g:0-255, b:0-255 }             — buffer all 12
 * - `rgb.refresh` {}                                        — latch to LEDs
 *
 * Colors are buffered until `rgb.refresh` is called.
 *
 * @param[in] node OpenClaw Node instance to extend.
 *
 * @return
 *      - `ESP_OK` on success
 *      - an ESP-IDF error code if registration fails
 */
esp_err_t esp_openclaw_node_stackchan_register_rgb_commands(esp_openclaw_node_handle_t node);

/**
 * @brief AXP2101 soft power-off — cuts all rails immediately (same end result
 *        as a completed long-press). Does not return on success.
 */
esp_err_t stackchan_power_off(void);

/**
 * @brief Graceful shutdown: recenter the head + release servo torque, then
 *        soft power-off via the AXP2101. Run this INSTEAD of a bare long-press
 *        so the servos come home before power drops.
 */
esp_err_t stackchan_shutdown_gracefully(void);

/**
 * @brief Register the serial `power off | off-hard` REPL command.
 */
esp_err_t stackchan_power_register_command(void);

/**
 * @brief Start the AXP2101 power-button monitor: a single SHORT press of the
 *        physical power key triggers a graceful shutdown (recenter servos +
 *        soft power-off). Long press still hard-cuts via AXP hardware.
 */
esp_err_t stackchan_power_button_monitor_start(void);
