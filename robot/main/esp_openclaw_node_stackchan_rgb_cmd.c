/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_openclaw_node_stackchan_rgb_cmd.h"

#include <string.h>

#include "cJSON.h"
#include "esp_check.h"
#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_openclaw_node_example_json.h"
#include "stackchan_servo.h"
#include "stackchan_body.h"  /* goodnight screen: avatar_set / display_goodnight */
#include "stackchan_presentation.h" /* quiesce coordinator on shutdown */
#include "stackchan_m5_i2c.h" /* shared-owner I2C + M5.Power (no second bus!)   */
#include "stackchan_voice.h"   /* touch-to-mic: toggle_listening / is_listening   */

static const char *TAG = "esp_openclaw_node_rgb";

/* All CoreS3-internal I2C devices below are reached through M5.In_I2C (the
 * SINGLE bus owner) via the stackchan_m5_i2c shim -- NOT a second raw
 * i2c_master bus. See stackchan_m5_i2c.h for the root-cause writeup: creating a
 * second owner on I2C_NUM_0 made M5's Speaker.begin() (AW88298) fail so the TTS
 * reply never played. Everything here now serializes on M5's own i2c lock. */
#define RGB_I2C_SPEED_HZ    100000

/* AW9523 IO-expander on the CoreS3 mainboard — gates 5V boost + external bus
 * power that feeds the StackChan robot base (where the PY32 + LEDs live). */
#define AW9523_ADDR         0x58
#define AW9523_REG_ID       0x10  /* reads 0x23 on a live AW9523B */
#define AW9523_REG_P0_OUT   0x02
#define AW9523_REG_P1_OUT   0x03
#define AW9523_REG_P0_CFG   0x04
#define AW9523_REG_P1_CFG   0x05
#define AW9523_REG_GCR      0x11
#define AW9523_REG_P0_MODE  0x12
#define AW9523_REG_P1_MODE  0x13
#define AW9523_BUS_EN       0x02  /* P0 bit1: external bus 5V enable          */
#define AW9523_BOOST_EN     0x80  /* P1 bit7: SY7088 boost enable             */

/* PY32L020 IO-expander on the robot base (drives the 12x WS2812C LEDs). */
#define PY32_ADDR           0x6F
#define PY32_LED_COUNT      12

/* AXP2101 PMIC. Soft power-off + power-key are handled through M5.Power (the
 * shared owner) -- we no longer poke AXP registers over a raw bus here. */

/* PY32 register map (from m5stack/StackChan-BSP PY32IOExpander.cpp). */
#define REG_UID_L           0x00
#define REG_VERSION         0x02
#define REG_GPIO_M_L        0x03  /* direction low  (1=output)               */
#define REG_GPIO_M_H        0x04  /* direction high                          */
#define REG_GPIO_O_L        0x05  /* output low                              */
#define REG_GPIO_PU_L       0x09  /* pull-up low                             */
#define REG_GPIO_PU_H       0x0A  /* pull-up high                            */
#define REG_GPIO_PD_L       0x0B  /* pull-down low                           */
#define REG_GPIO_PD_H       0x0C  /* pull-down high                          */
#define REG_GPIO_DRV_H      0x14  /* drive high (0=push-pull, 1=open-drain)  */
#define REG_LED_CFG         0x24  /* bits[5:0]=count, bit6=refresh/latch     */
#define REG_LED_RAM_START   0x30  /* 2 bytes/LED, RGB565, low byte first     */
#define REG_LED_REFRESH_BIT 0x40

#define PY32_VM_EN_PIN      0     /* expander pin 0: servo/VM power enable    */
#define PY32_RGB_PIN        13    /* expander pin 13: WS2812 data output      */

/* Cooperative pause counter for the shared internal I2C bus. Retained for the
 * LED transport in stackchan_boot.cpp (stackchan_led_fill) + the voice codec
 * BUS_LOCK, which still bump it. The power-key poller NO LONGER reads it: with
 * everything on M5.In_I2C the two-owner race is gone structurally, and the
 * poller's getKeyState() serializes at M5's own i2c lock. Left in place so the
 * existing LED/codec call sites compile and behave unchanged. */
volatile int g_stackchan_power_poll_pause = 0;

static bool s_initialized = false;

/* ------------------------------ I2C helpers ------------------------------ *
 * Addressed by 7-bit device address and routed through M5.In_I2C (the single
 * internal-bus owner). Same register sequences as before; only the transport
 * changed from a raw i2c_master bus to the shared owner. */
static esp_err_t dev_write8(uint8_t addr, uint8_t reg, uint8_t val)
{
    return stackchan_m5_i2c_write8(addr, reg, val, RGB_I2C_SPEED_HZ) ? ESP_OK : ESP_FAIL;
}

static esp_err_t dev_read8(uint8_t addr, uint8_t reg, uint8_t *out)
{
    return stackchan_m5_i2c_read8(addr, reg, out, RGB_I2C_SPEED_HZ) ? ESP_OK : ESP_FAIL;
}

static esp_err_t dev_bit_on(uint8_t addr, uint8_t reg, uint8_t mask)
{
    return stackchan_m5_i2c_bit_on(addr, reg, mask, RGB_I2C_SPEED_HZ) ? ESP_OK : ESP_FAIL;
}

static esp_err_t dev_write_led(uint8_t addr, uint8_t reg,
                               const uint8_t *data, size_t len)
{
    return stackchan_m5_i2c_write(addr, reg, data, len, RGB_I2C_SPEED_HZ) ? ESP_OK : ESP_FAIL;
}

/* --------------------------- power / expander ---------------------------- */
/* Bring up the AW9523 and switch on the 5V boost + external bus so the robot
 * base (and its PY32) actually get power. Mirrors M5GFX/M5Unified CoreS3.
 * NOTE: M5.begin(output_power=true) already powers the base at boot, so this is
 * an idempotent re-assert on the shared bus for the standalone rgb.* path. */
static esp_err_t aw9523_power_on(void)
{
    uint8_t id = 0;
    esp_err_t err = dev_read8(AW9523_ADDR, AW9523_REG_ID, &id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AW9523 not responding at 0x58");
        return err;
    }
    if (id != 0x23) {
        ESP_LOGW(TAG, "AW9523 ID 0x%02x (expected 0x23), continuing", id);
    }

    /* Port config: 1=input, 0=output (buttons stay inputs). */
    ESP_RETURN_ON_ERROR(dev_write8(AW9523_ADDR, AW9523_REG_GCR, 0x10), TAG, "gcr");     /* P0 push-pull */
    ESP_RETURN_ON_ERROR(dev_write8(AW9523_ADDR, AW9523_REG_P0_MODE, 0xFF), TAG, "p0mode"); /* GPIO mode */
    ESP_RETURN_ON_ERROR(dev_write8(AW9523_ADDR, AW9523_REG_P1_MODE, 0xFF), TAG, "p1mode");
    ESP_RETURN_ON_ERROR(dev_write8(AW9523_ADDR, AW9523_REG_P0_CFG, 0x18), TAG, "p0cfg");
    ESP_RETURN_ON_ERROR(dev_write8(AW9523_ADDR, AW9523_REG_P1_CFG, 0x0C), TAG, "p1cfg");

    /* Enable SY7088 boost (P1 bit7) then external bus 5V (P0 bit1). */
    ESP_RETURN_ON_ERROR(dev_bit_on(AW9523_ADDR, AW9523_REG_P1_OUT, AW9523_BOOST_EN), TAG, "boost");
    ESP_RETURN_ON_ERROR(dev_bit_on(AW9523_ADDR, AW9523_REG_P0_OUT, AW9523_BUS_EN), TAG, "bus");

    /* Give the base board + PY32 time to power up and boot. */
    vTaskDelay(pdMS_TO_TICKS(300));
    return ESP_OK;
}

/* PY32 pin config (matches M5's _writeBit high/low split). */
static esp_err_t py32_set_direction_out(uint8_t pin)
{
    return (pin < 8) ? dev_bit_on(PY32_ADDR, REG_GPIO_M_L, 1 << pin)
                     : dev_bit_on(PY32_ADDR, REG_GPIO_M_H, 1 << (pin - 8));
}
static esp_err_t py32_set_pullup(uint8_t pin)
{
    return (pin < 8) ? dev_bit_on(PY32_ADDR, REG_GPIO_PU_L, 1 << pin)
                     : dev_bit_on(PY32_ADDR, REG_GPIO_PU_H, 1 << (pin - 8));
}
static esp_err_t py32_output_high(uint8_t pin)
{
    return dev_bit_on(PY32_ADDR, REG_GPIO_O_L, 1 << pin);  /* low byte pins only */
}

static esp_err_t py32_enable_leds(void)
{
    uint8_t version = 0;
    /* PY32 may boot slowly after bus power comes up; poll up to ~1.2s. */
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 12; i++) {
        err = dev_read8(PY32_ADDR, REG_VERSION, &version);
        if (err == ESP_OK && version != 0x00 && version != 0xFF) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (err != ESP_OK || version == 0x00 || version == 0xFF) {
        ESP_LOGE(TAG, "PY32 not alive (version=0x%02x)", version);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "PY32 alive, version=0x%02x", version);

    /* VM_EN: power the servo/motor rail (pin 0 output high). */
    ESP_RETURN_ON_ERROR(py32_set_direction_out(PY32_VM_EN_PIN), TAG, "vm dir");
    ESP_RETURN_ON_ERROR(py32_set_pullup(PY32_VM_EN_PIN), TAG, "vm pu");
    ESP_RETURN_ON_ERROR(py32_output_high(PY32_VM_EN_PIN), TAG, "vm hi");

    /* RGB data pin: output, pull-up, push-pull. */
    ESP_RETURN_ON_ERROR(py32_set_direction_out(PY32_RGB_PIN), TAG, "rgb dir");
    ESP_RETURN_ON_ERROR(py32_set_pullup(PY32_RGB_PIN), TAG, "rgb pu");
    /* push-pull = clear drive-high bit5; read-modify-write */
    uint8_t drv = 0;
    ESP_RETURN_ON_ERROR(dev_read8(PY32_ADDR, REG_GPIO_DRV_H, &drv), TAG, "drv rd");
    ESP_RETURN_ON_ERROR(dev_write8(PY32_ADDR, REG_GPIO_DRV_H, drv & ~(1 << (PY32_RGB_PIN - 8))), TAG, "drv wr");

    /* LED count (bits[5:0]); refresh bit stays clear. */
    ESP_RETURN_ON_ERROR(dev_write8(PY32_ADDR, REG_LED_CFG, PY32_LED_COUNT & 0x3F), TAG, "count");
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

static esp_err_t rgb_ensure_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }
    /* No bus creation here anymore: M5.begin() already owns I2C_NUM_0 and
     * powered the base. We just (re-)assert the expander/LED config through the
     * shared owner so the rgb.* RPCs work standalone. */
    ESP_RETURN_ON_ERROR(aw9523_power_on(), TAG, "aw9523 power-on");
    ESP_RETURN_ON_ERROR(py32_enable_leds(), TAG, "py32 led enable");

    s_initialized = true;
    return ESP_OK;
}

/* RGB888 -> RGB565, low byte first (matches M5Stack's own conversion). */
static esp_err_t rgb_set_led(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t color565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    uint8_t data[2] = { color565 & 0xFF, (color565 >> 8) & 0xFF };
    return dev_write_led(PY32_ADDR, REG_LED_RAM_START + index * 2, data, sizeof(data));
}

static esp_err_t rgb_refresh(void)
{
    uint8_t cfg = 0;
    ESP_RETURN_ON_ERROR(dev_read8(PY32_ADDR, REG_LED_CFG, &cfg), TAG, "read cfg");
    return dev_write8(PY32_ADDR, REG_LED_CFG, cfg | REG_LED_REFRESH_BIT);
}

static bool parse_u8(cJSON *params, const char *key, uint8_t *out)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(params, key);
    if (!cJSON_IsNumber(item)) {
        return false;
    }
    int v = item->valueint;
    if (v < 0 || v > 255) {
        return false;
    }
    *out = (uint8_t)v;
    return true;
}

static esp_err_t handle_rgb_set(
    esp_openclaw_node_handle_t node, void *context,
    const char *params_json, size_t params_len,
    char **out_payload_json, esp_openclaw_node_error_t *out_error)
{
    (void)node; (void)context; (void)params_len;

    cJSON *params = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &params, out_error),
        TAG, "invalid params");

    cJSON *index = cJSON_GetObjectItemCaseSensitive(params, "index");
    uint8_t r, g, b;
    if (!cJSON_IsNumber(index) || index->valueint < 0 || index->valueint >= PY32_LED_COUNT) {
        out_error->message = "index must be 0-11";
        cJSON_Delete(params);
        return ESP_ERR_INVALID_ARG;
    }
    if (!parse_u8(params, "r", &r) || !parse_u8(params, "g", &g) || !parse_u8(params, "b", &b)) {
        out_error->message = "r, g, b must each be 0-255";
        cJSON_Delete(params);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = rgb_ensure_init();
    if (err != ESP_OK) {
        out_error->message = "RGB power-up/init failed (see serial log)";
        cJSON_Delete(params);
        return err;
    }
    err = rgb_set_led((uint8_t)index->valueint, r, g, b);
    if (err != ESP_OK) {
        out_error->message = "rgb.set I2C write failed";
        cJSON_Delete(params);
        return err;
    }

    cJSON *payload = cJSON_CreateObject();
    if (payload == NULL) { cJSON_Delete(params); return ESP_ERR_NO_MEM; }
    cJSON_AddNumberToObject(payload, "index", index->valueint);
    cJSON_AddNumberToObject(payload, "r", r);
    cJSON_AddNumberToObject(payload, "g", g);
    cJSON_AddNumberToObject(payload, "b", b);
    cJSON_AddBoolToObject(payload, "buffered", true);
    cJSON_Delete(params);
    return esp_openclaw_node_example_take_json_payload(payload, out_payload_json);
}

static esp_err_t handle_rgb_fill(
    esp_openclaw_node_handle_t node, void *context,
    const char *params_json, size_t params_len,
    char **out_payload_json, esp_openclaw_node_error_t *out_error)
{
    (void)node; (void)context; (void)params_len;

    cJSON *params = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &params, out_error),
        TAG, "invalid params");

    uint8_t r, g, b;
    if (!parse_u8(params, "r", &r) || !parse_u8(params, "g", &g) || !parse_u8(params, "b", &b)) {
        out_error->message = "r, g, b must each be 0-255";
        cJSON_Delete(params);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = rgb_ensure_init();
    if (err != ESP_OK) {
        out_error->message = "RGB power-up/init failed (see serial log)";
        cJSON_Delete(params);
        return err;
    }
    for (uint8_t i = 0; i < PY32_LED_COUNT; i++) {
        err = rgb_set_led(i, r, g, b);
        if (err != ESP_OK) {
            out_error->message = "rgb.fill I2C write failed";
            cJSON_Delete(params);
            return err;
        }
    }

    cJSON *payload = cJSON_CreateObject();
    if (payload == NULL) { cJSON_Delete(params); return ESP_ERR_NO_MEM; }
    cJSON_AddNumberToObject(payload, "count", PY32_LED_COUNT);
    cJSON_AddNumberToObject(payload, "r", r);
    cJSON_AddNumberToObject(payload, "g", g);
    cJSON_AddNumberToObject(payload, "b", b);
    cJSON_AddBoolToObject(payload, "buffered", true);
    cJSON_Delete(params);
    return esp_openclaw_node_example_take_json_payload(payload, out_payload_json);
}

static esp_err_t handle_rgb_refresh(
    esp_openclaw_node_handle_t node, void *context,
    const char *params_json, size_t params_len,
    char **out_payload_json, esp_openclaw_node_error_t *out_error)
{
    (void)node; (void)context; (void)params_json; (void)params_len;

    esp_err_t err = rgb_ensure_init();
    if (err != ESP_OK) {
        out_error->message = "RGB power-up/init failed (see serial log)";
        return err;
    }
    err = rgb_refresh();
    if (err != ESP_OK) {
        out_error->message = "rgb.refresh I2C write failed";
        return err;
    }

    cJSON *payload = cJSON_CreateObject();
    if (payload == NULL) { return ESP_ERR_NO_MEM; }
    cJSON_AddBoolToObject(payload, "refreshed", true);
    return esp_openclaw_node_example_take_json_payload(payload, out_payload_json);
}

/* Diagnostic: report AW9523 ID and PY32 version/UID so we can tell a live
 * chip from writing into the void. */
static esp_err_t handle_rgb_probe(
    esp_openclaw_node_handle_t node, void *context,
    const char *params_json, size_t params_len,
    char **out_payload_json, esp_openclaw_node_error_t *out_error)
{
    (void)node; (void)context; (void)params_json; (void)params_len;

    esp_err_t err = rgb_ensure_init();
    if (err != ESP_OK) {
        out_error->message = "RGB power-up/init failed (see serial log)";
        return err;
    }

    uint8_t aw_id = 0, ver = 0, uid_l = 0;
    bool aw_ok = (dev_read8(AW9523_ADDR, AW9523_REG_ID, &aw_id) == ESP_OK);
    bool py_ok = (dev_read8(PY32_ADDR, REG_VERSION, &ver) == ESP_OK);
    dev_read8(PY32_ADDR, REG_UID_L, &uid_l);

    cJSON *payload = cJSON_CreateObject();
    if (payload == NULL) { return ESP_ERR_NO_MEM; }
    cJSON_AddBoolToObject(payload, "aw9523_ok", aw_ok);
    cJSON_AddNumberToObject(payload, "aw9523_id", aw_id);
    cJSON_AddBoolToObject(payload, "py32_ok", py_ok && ver != 0 && ver != 0xFF);
    cJSON_AddNumberToObject(payload, "py32_version", ver);
    cJSON_AddNumberToObject(payload, "py32_uid_l", uid_l);
    return esp_openclaw_node_example_take_json_payload(payload, out_payload_json);
}

/* Soft power-off through M5.Power (the shared internal-I2C owner): cuts all
 * rails, exactly like a completed long-press. Does NOT return on success (the
 * chip powers down). No raw AXP bus involved anymore. */
esp_err_t stackchan_power_off(void)
{
    ESP_LOGW(TAG, "AXP2101 soft power-off (via M5.Power)");
    vTaskDelay(pdMS_TO_TICKS(20));
    stackchan_m5_power_off();
    return ESP_OK;  /* normally never reached; the rails drop */
}

/* Polls the AXP2101 power-key via M5.Power.getKeyState() (routes through
 * M5.In_I2C, the single internal-bus owner). A single SHORT press (return
 * value 2) -> graceful shutdown (goodnight + recenter + soft power-off). Long
 * press (1) is left to the AXP hardware hard-cut fallback. */
static void power_button_task(void *arg)
{
    (void)arg;
    /* BOOT GRACE: powering ON is itself a power-key press that latches a
     * short-press in the AXP2101. Wait for the boot+connect transient to pass,
     * then DRAIN the latched power-on press so we never treat it as a shutdown
     * request. getKeyState() reads AXP reg 0x49 (mask 0x0C) through the shared
     * owner and auto-clears the latch (W1C). */
    vTaskDelay(pdMS_TO_TICKS(6000));
    (void)stackchan_m5_get_key_state();  /* drain the power-on press */
    ESP_LOGI(TAG, "power-key monitor armed (M5.Power.getKeyState; short press = shutdown)");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(40));
        /* getKeyState(): 0=none, 1=long, 2=short, 3=both. It runs through
         * M5.In_I2C -- the SINGLE internal-I2C owner -- so it serializes with
         * the audio codec begin()/end() at lgfx's own i2c lock. There is no
         * second bus owner and no wedge, so a contended read returns 0 ("none",
         * via readRegister8 value_or(0)) instead of the garbage 0xFF that
         * caused the historic phantom shutdowns. We act ONLY on an exact short
         * press (2); a spurious/failed read can never be exactly 2. */
        uint8_t k = stackchan_m5_get_key_state();
        if (k == 2) {
            ESP_LOGW(TAG, "power key SHORT press -> graceful shutdown");
            stackchan_shutdown_gracefully();  /* goodnight + recenter + off (no return) */
        }

        /* Tap the touchscreen -> toggle the mic on/off (same PTT path the
         * remote stick-CLICK drives). Polled on this same serialized 40ms
         * loop so touch reads coexist with the power-key read on the single
         * internal bus. */
        if (stackchan_m5_touch_tapped()) {
            bool now_on = !stackchan_voice_is_listening();
            stackchan_voice_toggle_listening();
            ESP_LOGI(TAG, "touch tap -> mic %s", now_on ? "ON" : "OFF");
        }
    }
}

esp_err_t stackchan_power_button_monitor_start(void)
{
    static bool started = false;
    if (started) return ESP_OK;
    /* M5.begin() already owns the internal I2C bus + AXP2101; nothing to
     * ensure here. Just start the poller. */
    if (xTaskCreatePinnedToCore(power_button_task, "pwr_btn", 3072, NULL, 4, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    started = true;
    ESP_LOGI(TAG, "power-button monitor started (short press = graceful shutdown)");
    return ESP_OK;
}

/* Graceful shutdown: park the head at center + relax torque, then cut power.
 * This is what to run INSTEAD of a bare long-press so the servos come home. */
esp_err_t stackchan_shutdown_gracefully(void)
{
    ESP_LOGW(TAG, "graceful shutdown: goodnight screen -> recenter servos -> power off");
    /* 0. Tell the presentation coordinator to quiesce so it stops driving the
     *    head/LED ring and does not re-torque the servos after we relax them. */
    stackchan_presentation_note_shutdown();
    /* 1. Freeze the avatar + show the "goodnight" screen. LCD draw is SPI, so
     *    it adds no I2C traffic that could collide with anything. */
    stackchan_avatar_set(false);
    stackchan_display_goodnight();
    /* 2. Glide the head to dead center + release torque (~1.1s; screen is up
     *    the whole time). */
    stackchan_servo_shutdown();
    /* 3. Hold so "Goodnight" stays readable before the rails drop. */
    vTaskDelay(pdMS_TO_TICKS(1200));
    /* 4. Soft power-off via M5.Power -- cuts all rails (no return on success). */
    return stackchan_power_off();
}

static esp_err_t handle_power_off(
    esp_openclaw_node_handle_t node, void *context,
    const char *params_json, size_t params_len,
    char **out_payload_json, esp_openclaw_node_error_t *out_error)
{
    (void)node; (void)context; (void)params_json; (void)params_len; (void)out_error;
    /* Ack first so the gateway sees a reply before the rails drop. */
    cJSON *payload = cJSON_CreateObject();
    if (payload == NULL) { return ESP_ERR_NO_MEM; }
    cJSON_AddBoolToObject(payload, "shutting_down", true);
    esp_err_t take = esp_openclaw_node_example_take_json_payload(payload, out_payload_json);
    /* Kick the actual shutdown slightly deferred so this response can flush.
     * Same graceful sequence as a short press: goodnight -> recenter -> off. */
    stackchan_presentation_note_shutdown();
    stackchan_avatar_set(false);
    stackchan_display_goodnight();
    stackchan_servo_shutdown();
    stackchan_power_off();
    return take;
}

/* ------------------------------ serial REPL ------------------------------ */
static int handle_power_cmd(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "off") == 0) {
        printf("graceful shutdown: recentering servos then powering off...\n");
        stackchan_shutdown_gracefully();
        return 0;  /* usually never reached; the chip powers down */
    }
    if (argc == 2 && strcmp(argv[1], "off-hard") == 0) {
        printf("hard power-off (no servo recenter)...\n");
        stackchan_power_off();
        return 0;
    }
    printf("usage: power off | off-hard\n");
    return 1;
}

esp_err_t stackchan_power_register_command(void)
{
    esp_console_cmd_t cmd = {};
    cmd.command = "power";
    cmd.help = "Graceful shutdown: recenter servos + AXP2101 soft power-off.";
    cmd.hint = "off|off-hard";
    cmd.func = handle_power_cmd;
    return esp_console_cmd_register(&cmd);
}

esp_err_t esp_openclaw_node_stackchan_register_rgb_commands(esp_openclaw_node_handle_t node)
{
    static const esp_openclaw_node_command_t RGB_SET_COMMAND = {
        .name = "rgb.set", .handler = handle_rgb_set, .context = NULL,
    };
    static const esp_openclaw_node_command_t RGB_FILL_COMMAND = {
        .name = "rgb.fill", .handler = handle_rgb_fill, .context = NULL,
    };
    static const esp_openclaw_node_command_t RGB_REFRESH_COMMAND = {
        .name = "rgb.refresh", .handler = handle_rgb_refresh, .context = NULL,
    };
    static const esp_openclaw_node_command_t RGB_PROBE_COMMAND = {
        .name = "rgb.probe", .handler = handle_rgb_probe, .context = NULL,
    };
    static const esp_openclaw_node_command_t POWER_OFF_COMMAND = {
        .name = "power.off", .handler = handle_power_off, .context = NULL,
    };

    ESP_RETURN_ON_ERROR(esp_openclaw_node_register_capability(node, "rgb"), TAG, "cap rgb");
    ESP_RETURN_ON_ERROR(esp_openclaw_node_register_command(node, &RGB_SET_COMMAND), TAG, "rgb.set");
    ESP_RETURN_ON_ERROR(esp_openclaw_node_register_command(node, &RGB_FILL_COMMAND), TAG, "rgb.fill");
    ESP_RETURN_ON_ERROR(esp_openclaw_node_register_command(node, &RGB_REFRESH_COMMAND), TAG, "rgb.refresh");
    ESP_RETURN_ON_ERROR(esp_openclaw_node_register_command(node, &RGB_PROBE_COMMAND), TAG, "rgb.probe");
    ESP_RETURN_ON_ERROR(esp_openclaw_node_register_command(node, &POWER_OFF_COMMAND), TAG, "power.off");
    return ESP_OK;
}
