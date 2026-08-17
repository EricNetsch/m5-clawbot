/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_openclaw_node_example_gpio_node_cmd.h"

#include <string.h>

#include "cJSON.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_openclaw_node_example_json.h"

static const char *TAG = "esp_openclaw_node_gpio";
static bool s_gpio_mode_configured[GPIO_NUM_MAX];
static gpio_mode_t s_gpio_modes[GPIO_NUM_MAX];

static bool gpio_mode_supports_output(gpio_mode_t mode)
{
    return mode == GPIO_MODE_OUTPUT || mode == GPIO_MODE_INPUT_OUTPUT;
}

static esp_err_t parse_required_pin(cJSON *params, gpio_num_t *out_pin, bool require_output)
{
    cJSON *pin = cJSON_GetObjectItemCaseSensitive(params, "pin");
    if (!cJSON_IsNumber(pin)) {
        return ESP_ERR_INVALID_ARG;
    }
    int pin_num = pin->valueint;
    if (!GPIO_IS_VALID_GPIO(pin_num) ||
        (require_output && !GPIO_IS_VALID_OUTPUT_GPIO(pin_num))) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_pin = (gpio_num_t)pin_num;
    return ESP_OK;
}

static esp_err_t handle_gpio_mode(
    esp_openclaw_node_handle_t node,
    void *context,
    const char *params_json,
    size_t params_len,
    char **out_payload_json,
    esp_openclaw_node_error_t *out_error)
{
    (void)node;
    (void)context;
    (void)params_len;

    cJSON *params = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &params, out_error),
        TAG,
        "invalid params");

    gpio_num_t pin = GPIO_NUM_NC;
    if (parse_required_pin(params, &pin, false) != ESP_OK) {
        out_error->message = "invalid GPIO pin";
        cJSON_Delete(params);
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *mode = cJSON_GetObjectItemCaseSensitive(params, "mode");
    if (!cJSON_IsString(mode) || mode->valuestring == NULL) {
        out_error->message = "mode must be one of input, output, input_output";
        cJSON_Delete(params);
        return ESP_ERR_INVALID_ARG;
    }

    gpio_mode_t gpio_mode = GPIO_MODE_DISABLE;
    if (strcmp(mode->valuestring, "input") == 0) {
        gpio_mode = GPIO_MODE_INPUT;
    } else if (strcmp(mode->valuestring, "output") == 0) {
        gpio_mode = GPIO_MODE_OUTPUT;
    } else if (strcmp(mode->valuestring, "input_output") == 0) {
        gpio_mode = GPIO_MODE_INPUT_OUTPUT;
    } else {
        out_error->message = "mode must be one of input, output, input_output";
        cJSON_Delete(params);
        return ESP_ERR_INVALID_ARG;
    }

    bool pull_up = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(params, "pullUp"));
    bool pull_down = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(params, "pullDown"));

    gpio_config_t config = {
        .pin_bit_mask = 1ULL << pin,
        .mode = gpio_mode,
        .pull_up_en = pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = pull_down ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) {
        cJSON_Delete(params);
        ESP_RETURN_ON_ERROR(err, TAG, "gpio_config failed");
    }
    s_gpio_modes[pin] = gpio_mode;
    s_gpio_mode_configured[pin] = true;

    cJSON *payload = cJSON_CreateObject();
    if (payload == NULL) {
        cJSON_Delete(params);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddNumberToObject(payload, "pin", pin);
    cJSON_AddStringToObject(payload, "mode", mode->valuestring);
    cJSON_AddBoolToObject(payload, "pullUp", pull_up);
    cJSON_AddBoolToObject(payload, "pullDown", pull_down);
    cJSON_Delete(params);
    return esp_openclaw_node_example_take_json_payload(payload, out_payload_json);
}

static esp_err_t handle_gpio_read(
    esp_openclaw_node_handle_t node,
    void *context,
    const char *params_json,
    size_t params_len,
    char **out_payload_json,
    esp_openclaw_node_error_t *out_error)
{
    (void)node;
    (void)context;
    (void)params_len;

    cJSON *params = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &params, out_error),
        TAG,
        "invalid params");

    gpio_num_t pin = GPIO_NUM_NC;
    if (parse_required_pin(params, &pin, false) != ESP_OK) {
        out_error->message = "invalid GPIO pin";
        cJSON_Delete(params);
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *payload = cJSON_CreateObject();
    if (payload == NULL) {
        cJSON_Delete(params);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddNumberToObject(payload, "pin", pin);
    cJSON_AddNumberToObject(payload, "level", gpio_get_level(pin));
    cJSON_Delete(params);
    return esp_openclaw_node_example_take_json_payload(payload, out_payload_json);
}

static esp_err_t handle_gpio_write(
    esp_openclaw_node_handle_t node,
    void *context,
    const char *params_json,
    size_t params_len,
    char **out_payload_json,
    esp_openclaw_node_error_t *out_error)
{
    (void)node;
    (void)context;
    (void)params_len;

    cJSON *params = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &params, out_error),
        TAG,
        "invalid params");

    gpio_num_t pin = GPIO_NUM_NC;
    if (parse_required_pin(params, &pin, true) != ESP_OK) {
        out_error->message = "invalid output GPIO pin";
        cJSON_Delete(params);
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *level = cJSON_GetObjectItemCaseSensitive(params, "level");
    if (!cJSON_IsNumber(level) && !cJSON_IsBool(level)) {
        out_error->message = "level must be 0, 1, true, or false";
        cJSON_Delete(params);
        return ESP_ERR_INVALID_ARG;
    }

    int level_value = cJSON_IsTrue(level) ? 1 : level->valueint ? 1 : 0;
    if (!s_gpio_mode_configured[pin] || !gpio_mode_supports_output(s_gpio_modes[pin])) {
        out_error->message = "configure gpio.mode with output or input_output before gpio.write";
        cJSON_Delete(params);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = gpio_set_level(pin, level_value);
    if (err != ESP_OK) {
        cJSON_Delete(params);
        ESP_RETURN_ON_ERROR(err, TAG, "gpio_set_level failed");
    }

    cJSON *payload = cJSON_CreateObject();
    if (payload == NULL) {
        cJSON_Delete(params);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddNumberToObject(payload, "pin", pin);
    cJSON_AddNumberToObject(payload, "level", level_value);
    cJSON_Delete(params);
    return esp_openclaw_node_example_take_json_payload(payload, out_payload_json);
}

esp_err_t esp_openclaw_node_example_register_gpio_node_commands(esp_openclaw_node_handle_t node)
{
    memset(s_gpio_mode_configured, 0, sizeof(s_gpio_mode_configured));
    memset(s_gpio_modes, 0, sizeof(s_gpio_modes));

    static const esp_openclaw_node_command_t GPIO_MODE_COMMAND = {
        .name = "gpio.mode",
        .handler = handle_gpio_mode,
        .context = NULL,
    };
    static const esp_openclaw_node_command_t GPIO_READ_COMMAND = {
        .name = "gpio.read",
        .handler = handle_gpio_read,
        .context = NULL,
    };
    static const esp_openclaw_node_command_t GPIO_WRITE_COMMAND = {
        .name = "gpio.write",
        .handler = handle_gpio_write,
        .context = NULL,
    };

    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_register_capability(node, "gpio"),
        TAG,
        "registering gpio capability failed");
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_register_command(node, &GPIO_MODE_COMMAND),
        TAG,
        "registering gpio.mode failed");
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_register_command(node, &GPIO_READ_COMMAND),
        TAG,
        "registering gpio.read failed");
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_register_command(node, &GPIO_WRITE_COMMAND),
        TAG,
        "registering gpio.write failed");
    return ESP_OK;
}
