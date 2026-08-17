/*
 * JoyC reader. Mirrors the factory joystick_handle.c VERBATIM:
 *   - called AFTER M5.begin()
 *   - i2c_bus_create(I2C_NUM_0, G0=SDA/G26=SCL, 100kHz) ONCE
 *   - i2c_bus_scan() for logging
 *   - i2c_bus_device_create(bus, 0x54, 0)
 *   - read reg 0x00 [2B] -> 20ms -> reg 0x02 [2B]; button reg 0x30, active-low.
 *
 * Config MUST match factory: CONFIG_I2C_BUS_BACKWARD_CONFIG NOT set (i2c_bus
 * rides the new i2c_master driver). Do NOT do pre-M5 scans, delete/recreate the
 * bus, or probe multiple pin pairs -- any of those triggers the
 * "CONFLICT! driver_ng" abort. One bus, one device, exactly like the factory.
 *
 * IMPORTANT: include only i2c_bus.h here. Do NOT include driver/i2c.h or
 * M5Unified.h in this file (i2c_config_t conflict; see joyc.h).
 */
#include "joyc.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"   /* GPIO_PULLUP_ENABLE */
#include "i2c_bus.h"

#define JOYC_ADDR 0x54
#define REG_X     0x00 /* 2 bytes, 16-bit LE ADC */
#define REG_Y     0x02 /* 2 bytes, 16-bit LE ADC */
#define REG_BTN   0x30 /* 1 byte, active-low (0 = pressed) */

#define JOYC_SDA  0    /* HAT header SDA (factory default) */
#define JOYC_SCL  26   /* HAT header SCL */

static const char *TAG = "joyc";

static i2c_bus_handle_t        s_bus  = NULL;
static i2c_bus_device_handle_t s_joyc = NULL;
static bool                    s_ok   = false;

bool joyc_init(void)
{
    i2c_config_t conf;
    conf.mode          = I2C_MODE_MASTER;
    conf.sda_io_num    = JOYC_SDA;
    conf.scl_io_num    = JOYC_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    conf.clk_flags        = 0;

    s_bus = i2c_bus_create(I2C_NUM_0, &conf);
    if (s_bus == NULL) {
        ESP_LOGE(TAG, "i2c_bus_create(NUM_0, SDA=%d SCL=%d) FAILED", JOYC_SDA, JOYC_SCL);
        return false;
    }

    /* Factory-style bus scan for logging. */
    uint8_t found[128];
    memset(found, 0, sizeof(found));
    i2c_bus_scan(s_bus, found, sizeof(found));
    for (size_t i = 0; i < sizeof(found); i++) {
        if (found[i] != 0) ESP_LOGI(TAG, "scan: device @ 0x%02X", found[i]);
    }

    s_joyc = i2c_bus_device_create(s_bus, JOYC_ADDR, 0);

    uint8_t d[4] = {0};
    esp_err_t r = ESP_FAIL;
    if (s_joyc) {
        r = i2c_bus_read_bytes(s_joyc, REG_X, 2, d);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        r |= i2c_bus_read_bytes(s_joyc, REG_Y, 2, &d[2]);
    }
    s_ok = (r == ESP_OK);
    ESP_LOGI(TAG, "JoyC init SDA=%d SCL=%d addr=0x%02X -> ok=%d x=%u y=%u",
             JOYC_SDA, JOYC_SCL, JOYC_ADDR, (int)s_ok,
             (unsigned)((d[1] << 8) | d[0]), (unsigned)((d[3] << 8) | d[2]));
    if (!s_ok) ESP_LOGE(TAG, "JoyC read failed -- check the hat is seated");
    return s_ok;
}

bool joyc_read(uint16_t *x, uint16_t *y, bool *click)
{
    if (!s_joyc) return false;
    uint8_t d[4] = {0};
    esp_err_t r = i2c_bus_read_bytes(s_joyc, REG_X, 2, d);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    r |= i2c_bus_read_bytes(s_joyc, REG_Y, 2, &d[2]);
    if (r != ESP_OK) {
        s_ok = false;
        return false;
    }
    if (x) *x = (uint16_t)((d[1] << 8) | d[0]);
    if (y) *y = (uint16_t)((d[3] << 8) | d[2]);
    uint8_t b = 1; /* active-low: 1 = released */
    if (click && i2c_bus_read_bytes(s_joyc, REG_BTN, 1, &b) == ESP_OK) {
        *click = (b == 0);
    }
    s_ok = true;
    return true;
}

bool joyc_ok(void)
{
    return s_ok;
}
