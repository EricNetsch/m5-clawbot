/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"
#include "esp_openclaw_node.h"

/**
 * @brief Register the ADC command used by the generic ESP32 example.
 *
 * On ADC-capable targets the helper adds `adc.read`. Targets without ADC
 * support compile a no-op implementation that returns `ESP_OK`.
 *
 * @param[in] node OpenClaw Node instance to extend.
 *
 * @return
 *      - `ESP_OK` on success
 *      - an ESP-IDF error code if registration fails
 */
esp_err_t esp_openclaw_node_example_register_adc_node_commands(esp_openclaw_node_handle_t node);
