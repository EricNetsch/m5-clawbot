/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"
#include "esp_openclaw_node.h"

/**
 * @brief Register the generic ESP32 example command set.
 *
 * The helper adds the shared `device.*` and `wifi.status` commands plus the
 * example-specific GPIO and ADC commands supported by the selected target.
 *
 * @param[in] node OpenClaw Node instance to extend.
 *
 * @return
 *      - `ESP_OK` on success
 *      - an ESP-IDF error code if registration fails
 */
esp_err_t esp_openclaw_node_example_register_node_commands(esp_openclaw_node_handle_t node);
