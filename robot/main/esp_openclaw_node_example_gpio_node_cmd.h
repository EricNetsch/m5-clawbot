/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"
#include "esp_openclaw_node.h"

/**
 * @brief Register the GPIO commands used by the generic ESP32 example.
 *
 * The helper adds the `gpio.mode`, `gpio.read`, and `gpio.write` commands.
 * The documented path is:
 * - `gpio.mode` with `"input"`, `"output"`, or `"input_output"`
 * - `gpio.read`
 * - `gpio.write` after `"output"` or `"input_output"`
 *
 * Open-drain drive modes are intentionally not exposed in this helper.
 *
 * @param[in] node OpenClaw Node instance to extend.
 *
 * @return
 *      - `ESP_OK` on success
 *      - an ESP-IDF error code if registration fails
 */
esp_err_t esp_openclaw_node_example_register_gpio_node_commands(esp_openclaw_node_handle_t node);
