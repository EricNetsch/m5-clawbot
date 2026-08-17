/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_openclaw_node.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- Shared body control (implemented in stackchan_boot.cpp) --- */

/**
 * @brief Start/stop the looping avatar video on the LCD.
 *
 * When stopped, the display is left free for static drawing (text/clear).
 * When started, the avatar frames resume looping.
 */
void stackchan_avatar_set(bool playing);

/** @brief Fill all 12 status LEDs with an RGB color and latch it. */
void stackchan_led_fill(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Enter/exit the "thinking" (waiting-for-reply) state: pink pulsing
 *        LEDs + a pulsing pink orb on the LCD (or /spiffs/thinking.jpg if
 *        present). Call true on PTT-release, false when reply audio starts.
 */
void stackchan_thinking_set(bool on);

/** @brief True while the "thinking"/waiting-for-reply state is active. */
bool stackchan_thinking_active(void);

/** @brief Clear the LCD to a solid RGB565-packed color (pauses avatar). */
void stackchan_display_clear(uint16_t color565);

/** @brief Draw centered text (pauses avatar). size 1-8, RGB565 color. */
void stackchan_display_text(const char *text, uint8_t size, uint16_t color565);

/**
 * @brief Draw the shutdown "goodnight" screen (SPI/LCD only, no I2C). Uses
 *        /spiffs/goodnight.jpg if present, otherwise centered "Goodnight"
 *        text. Stops the thinking orb + avatar loop first so nothing draws
 *        over it. Safe to call from the power-button task.
 */
void stackchan_display_goodnight(void);

/** @brief Set LCD backlight brightness (0-255). */
void stackchan_display_brightness(uint8_t level);

/** @brief Set master speaker output volume (0-255). Used by remote TWEAK mode. */
void stackchan_set_volume(uint8_t level);

/** @brief Current master speaker output volume (0-255). */
uint8_t stackchan_get_volume(void);

/**
 * @brief Put the LCD panel to sleep (true) or wake it (false). SPI only -- no
 *        I2C traffic. Used by the idle/sleep tier to power the panel down at
 *        light idle and bring it back on wake. Pair with
 *        stackchan_display_brightness(0)/restore for the backlight.
 */
void stackchan_display_sleep(bool sleep);

/**
 * @brief Register the StackChan body commands (led/display/speaker) on the
 *        OpenClaw node. Call once after the node is created.
 */
esp_err_t esp_openclaw_node_stackchan_register_body_commands(esp_openclaw_node_handle_t node);

#ifdef __cplusplus
}
#endif
