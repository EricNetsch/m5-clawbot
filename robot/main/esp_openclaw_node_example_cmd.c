/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_openclaw_node_example_cmd.h"

#include "esp_openclaw_node_common_device_node_cmd.h"
#include "esp_check.h"
#include "soc/soc_caps.h"
#include "esp_openclaw_node_example_adc_node_cmd.h"
#include "esp_openclaw_node_example_gpio_node_cmd.h"
#include "esp_openclaw_node_stackchan_rgb_cmd.h"

static const char *TAG = "esp_openclaw_node_example";

esp_err_t esp_openclaw_node_example_register_node_commands(esp_openclaw_node_handle_t node)
{
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_common_register_device_node_commands(node),
        TAG,
        "registering device commands failed");
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_register_gpio_node_commands(node),
        TAG,
        "registering GPIO commands failed");
#if SOC_ADC_SUPPORTED
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_register_adc_node_commands(node),
        TAG,
        "registering ADC commands failed");
#endif
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_stackchan_register_rgb_commands(node),
        TAG,
        "registering RGB commands failed");
    return ESP_OK;
}
