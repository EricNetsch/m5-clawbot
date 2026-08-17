/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_openclaw_node_example_adc_node_cmd.h"

#include <stdbool.h>
#include <string.h>

#include "cJSON.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_openclaw_node_example_json.h"
#include "soc/adc_channel.h"
#include "soc/soc_caps.h"

#if SOC_ADC_SUPPORTED

typedef struct {
    adc_oneshot_unit_handle_t adc_handle;
    bool adc_unit_ready;
    bool adc_channel_ready[SOC_ADC_CHANNEL_NUM(0)];
    adc_cali_handle_t adc_cali[SOC_ADC_CHANNEL_NUM(0)];
    bool adc_cali_ready[SOC_ADC_CHANNEL_NUM(0)];
} esp_openclaw_node_example_adc_context_t;

static const char *TAG = "esp_openclaw_node_adc";
static esp_openclaw_node_example_adc_context_t s_adc;

static esp_err_t ensure_adc_channel(
    esp_openclaw_node_example_adc_context_t *context,
    adc_channel_t channel)
{
    if (!context->adc_unit_ready) {
        adc_oneshot_unit_init_cfg_t init = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init, &context->adc_handle), TAG, "adc init failed");
        context->adc_unit_ready = true;
    }

    if (!context->adc_channel_ready[channel]) {
        adc_oneshot_chan_cfg_t config = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ESP_RETURN_ON_ERROR(
            adc_oneshot_config_channel(context->adc_handle, channel, &config),
            TAG,
            "adc channel config failed");
        context->adc_channel_ready[channel] = true;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_curve_fitting_config_t cali = {
            .unit_id = ADC_UNIT_1,
            .chan = channel,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_curve_fitting(&cali, &context->adc_cali[channel]) == ESP_OK) {
            context->adc_cali_ready[channel] = true;
        }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_line_fitting_config_t cali = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_line_fitting(&cali, &context->adc_cali[channel]) == ESP_OK) {
            context->adc_cali_ready[channel] = true;
        }
#endif
    }

    return ESP_OK;
}

static esp_err_t handle_adc_read(
    esp_openclaw_node_handle_t node,
    void *context,
    const char *params_json,
    size_t params_len,
    char **out_payload_json,
    esp_openclaw_node_error_t *out_error)
{
    (void)node;
    (void)params_len;

    esp_openclaw_node_example_adc_context_t *adc = (esp_openclaw_node_example_adc_context_t *)context;
    cJSON *params = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &params, out_error),
        TAG,
        "invalid params");
    cJSON *channel = cJSON_GetObjectItemCaseSensitive(params, "channel");
    if (!cJSON_IsNumber(channel)) {
        out_error->message = "channel must be an integer";
        cJSON_Delete(params);
        return ESP_ERR_INVALID_ARG;
    }

    int channel_num = channel->valueint;
    if (channel_num < 0 || channel_num >= SOC_ADC_CHANNEL_NUM(0)) {
        out_error->message = "channel is out of range for ADC1";
        cJSON_Delete(params);
        return ESP_ERR_INVALID_ARG;
    }

    adc_channel_t adc_channel = (adc_channel_t)channel_num;
    esp_err_t err = ensure_adc_channel(adc, adc_channel);
    if (err != ESP_OK) {
        cJSON_Delete(params);
        ESP_RETURN_ON_ERROR(err, TAG, "adc channel setup failed");
    }

    int raw = 0;
    err = adc_oneshot_read(adc->adc_handle, adc_channel, &raw);
    if (err != ESP_OK) {
        cJSON_Delete(params);
        ESP_RETURN_ON_ERROR(err, TAG, "adc read failed");
    }

    cJSON *payload = cJSON_CreateObject();
    if (payload == NULL) {
        cJSON_Delete(params);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddNumberToObject(payload, "unit", 1);
    cJSON_AddNumberToObject(payload, "channel", channel_num);
    cJSON_AddNumberToObject(payload, "raw", raw);
    int gpio_num = -1;
    if (adc_oneshot_channel_to_io(ADC_UNIT_1, adc_channel, &gpio_num) == ESP_OK) {
        cJSON_AddNumberToObject(payload, "gpio", gpio_num);
    }
    if (adc->adc_cali_ready[channel_num]) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(adc->adc_cali[channel_num], raw, &mv) == ESP_OK) {
            cJSON_AddNumberToObject(payload, "millivolts", mv);
        }
    }
    cJSON_Delete(params);
    return esp_openclaw_node_example_take_json_payload(payload, out_payload_json);
}

esp_err_t esp_openclaw_node_example_register_adc_node_commands(esp_openclaw_node_handle_t node)
{
    static const esp_openclaw_node_command_t ADC_READ_COMMAND = {
        .name = "adc.read",
        .handler = handle_adc_read,
        .context = &s_adc,
    };

    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_register_capability(node, "adc"),
        TAG,
        "registering adc capability failed");
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_register_command(node, &ADC_READ_COMMAND),
        TAG,
        "registering adc.read failed");
    return ESP_OK;
}

#else

esp_err_t esp_openclaw_node_example_register_adc_node_commands(esp_openclaw_node_handle_t node)
{
    (void)node;
    return ESP_OK;
}

#endif
