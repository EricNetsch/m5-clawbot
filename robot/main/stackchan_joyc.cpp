/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * M5 Mini JoyC (U156) live head control.
 *
 * The Mini JoyC is an I2C joystick+button unit. On the CoreS3 base it plugs
 * into Port A (red Grove) -> M5Unified's external I2C bus (M5.Ex_I2C).
 *
 * I2C register map (from m5stack/M5Hat-Mini-JoyC library, same silicon/FW):
 *   7-bit address        0x54  (VERIFIED via that library's MiniJoyC_ADDR)
 *   ADC raw (16-bit LE)  0x00 = X, 0x02 = Y   (0..4095)
 *   POS 10-bit (LE)      0x10 = X, 0x12 = Y   (0..1023, firmware-calibrated)
 *   POS 8-bit            0x20 = X, 0x21 = Y   (0..255,  firmware-calibrated, ~128 center)
 *   BUTTON               0x30 (1 byte)
 *   RGB LED              0x40
 *   FW version           0xFE
 *   I2C addr cfg         0xFF
 *
 * We use the 8-bit calibrated POS registers (0x20/0x21): one byte each, already
 * mapped/centered by the unit firmware, plenty of resolution for head aiming.
 * To be robust against any calibration offset we also capture the resting
 * center at boot and normalize relative to that.
 */

#include "stackchan_joyc.h"
#include "stackchan_servo.h"
#include "stackchan_voice.h"
#include "stackchan_camera.h"

#include "esp_timer.h"

#include <M5Unified.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "stackchan_joyc";

/* ---- Mini JoyC I2C definitions ---- */
static constexpr uint8_t JOYC_ADDR = 0x54;      /* primary (per M5 library) */
static constexpr uint8_t JOYC_ADDR_ALT = 0x52;  /* fallback if re-addressed */
static constexpr uint32_t JOYC_FREQ = 100000;   /* conservative for Grove cables */
static constexpr uint8_t REG_POS8_X = 0x20;
static constexpr uint8_t REG_POS8_Y = 0x21;
static constexpr uint8_t REG_BUTTON = 0x30;
static constexpr uint8_t REG_FW_VER = 0xFE;

/* ---- Control tuning ---- */
static constexpr float DEADZONE = 0.14f;   /* ~14% resting deadzone */
static constexpr float POS_SPAN = 118.0f;  /* counts from center to full deflection */
static constexpr int YAW_RANGE_DEG = 30;   /* full-scale yaw (servo clamps ~+-31) */
static constexpr int PITCH_RANGE_DEG = 25; /* full-scale pitch */
static constexpr int MOVE_SPEED = 900;     /* 0..1000, high for responsive tracking */
static constexpr int SEND_THRESH_DEG = 2;  /* only resend when target moved this much */

/* ---- Mic-on-stick-DOWN tuning ----
 * Pull the stick DOWN to open the mic; let it return toward center to close it.
 * Intentionally sloppy on BOTH ends (wide hysteresis band) so it triggers over
 * a broad range of down-deflection and closes over a broad center zone -- no
 * precise stick placement required.
 *   ON  : down-deflection exceeds MIC_ON_THRESH (small -> easy to trigger).
 *   OFF : down-deflection falls back under MIC_OFF_THRESH (wide center zone).
 * The gap between the two is the hysteresis that prevents flicker at the edge. */
static constexpr float MIC_ON_THRESH  = 0.30f; /* ~30% down opens the mic          */
static constexpr float MIC_OFF_THRESH = 0.12f; /* back under ~12% down closes it   */
/* Which raw Y direction is "down". Y_SIGN maps up-stick -> +ny, so pulling DOWN
 * is -ny; down-amount = -ny. Flip to -1.0 if the stick feels inverted on-bench. */
static constexpr float JOYC_DOWN_SIGN = +1.0f;

/* Sign conventions (flip if the physical stick feels inverted):
 *   X: stick right -> larger POS value -> yaw right (positive). */
static constexpr float JOYC_X_SIGN = +1.0f;
/*   Y: stick up. Many M5 joysticks report a *larger* value when pushed up, and
 *   head-up should be pitch positive, so +1 maps up-stick -> head up. If it is
 *   reversed on the bench, flip this to -1. */
static constexpr float JOYC_Y_SIGN = +1.0f;

static uint8_t s_addr = JOYC_ADDR;
static float s_center_x = 128.0f;
static float s_center_y = 128.0f;
static int   s_btn_rest = 1;   /* button register value when NOTHING is pressed */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Read one 8-bit register; returns -1 on I2C failure. */
static int joyc_read8(uint8_t reg)
{
    uint8_t v = 0;
    if (!M5.Ex_I2C.readRegister(s_addr, reg, &v, 1, JOYC_FREQ)) {
        return -1;
    }
    return v;
}

/* Log every 7-bit device that ACKs on the external bus (debug aid). */
static void joyc_scan_bus(void)
{
    ESP_LOGI(TAG, "scanning Ex_I2C (Port A) ...");
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (M5.Ex_I2C.scanID(a, JOYC_FREQ)) {
            ESP_LOGI(TAG, "  found device at 0x%02X", a);
        }
    }
}

static bool joyc_probe(void)
{
    if (M5.Ex_I2C.scanID(JOYC_ADDR, JOYC_FREQ)) {
        s_addr = JOYC_ADDR;
        return true;
    }
    if (M5.Ex_I2C.scanID(JOYC_ADDR_ALT, JOYC_FREQ)) {
        s_addr = JOYC_ADDR_ALT;
        ESP_LOGW(TAG, "JoyC responded on fallback addr 0x%02X", JOYC_ADDR_ALT);
        return true;
    }
    return false;
}

static void joyc_capture_center(void)
{
    float sx = 0, sy = 0;
    int n = 0;
    for (int i = 0; i < 8; i++) {
        int x = joyc_read8(REG_POS8_X);
        int y = joyc_read8(REG_POS8_Y);
        if (x >= 0 && y >= 0) {
            sx += x;
            sy += y;
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (n > 0) {
        s_center_x = sx / n;
        s_center_y = sy / n;
    }
    /* Capture the button's RESTING value so we don't hardcode polarity. Whatever
     * it reads with nothing pressed is "released"; anything else is "pressed".
     * Take the mode over several reads to shrug off a noisy sample. */
    int zeros = 0, total = 0;
    for (int i = 0; i < 8; i++) {
        int b = joyc_read8(REG_BUTTON);
        if (b >= 0) { total++; if (b == 0) zeros++; }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (total > 0) {
        s_btn_rest = (zeros > total / 2) ? 0 : 1;
    }
    ESP_LOGI(TAG, "JoyC resting center: x=%.1f y=%.1f  btn_rest=%d",
             s_center_x, s_center_y, s_btn_rest);
}

static void joyc_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(40); /* ~25 Hz */
    bool prev_pressed = false; /* debounced pressed-state from last loop */
    int64_t last_click_us = 0; /* debounce for the side-button camera trigger */
    bool mic_open = false;     /* current mic state driven by stick-down          */

    ESP_LOGI(TAG, "JoyC control task running (addr 0x%02X)", s_addr);

    for (;;) {
        int xr = joyc_read8(REG_POS8_X);
        int yr = joyc_read8(REG_POS8_Y);

        if (xr >= 0 && yr >= 0) {
            float ny = clampf(JOYC_Y_SIGN * (yr - s_center_y) / POS_SPAN, -1.0f, 1.0f);
            /* down-amount: positive when the stick is pulled DOWN. */
            float down = JOYC_DOWN_SIGN * (-ny);

            /* ---- Servo head-aim from the stick: DISABLED for now ----
             * (Will be reworked into a deliberate "move" gesture later.)
            float nx = clampf(JOYC_X_SIGN * (xr - s_center_x) / POS_SPAN, -1.0f, 1.0f);
            float mag = sqrtf(nx * nx + ny * ny);
            if (mag < DEADZONE) {
                if (active) {
                    stackchan_servo_home();
                    stackchan_servo_idle_set(true);
                    active = false;
                    last_yaw = last_pitch = 0;
                }
            } else {
                int yaw_deg = (int)lroundf(nx * YAW_RANGE_DEG);
                int pitch_deg = (int)lroundf(ny * PITCH_RANGE_DEG);
                if (!active ||
                    abs(yaw_deg - last_yaw) >= SEND_THRESH_DEG ||
                    abs(pitch_deg - last_pitch) >= SEND_THRESH_DEG) {
                    stackchan_servo_move(yaw_deg, pitch_deg, MOVE_SPEED);
                    last_yaw = yaw_deg;
                    last_pitch = pitch_deg;
                    active = true;
                }
            }
            */

            /* ---- Mic on stick-DOWN (wide hysteresis, sloppy on both ends) ----
             * Pull down past MIC_ON_THRESH to open the mic; let it fall back
             * under MIC_OFF_THRESH (a wide center zone) to close it. Drives the
             * voice listening state directly (no toggle-edge needed). */
            if (!mic_open && down > MIC_ON_THRESH) {
                mic_open = true;
                if (!stackchan_voice_is_listening()) {
                    stackchan_voice_toggle_listening();
                }
                ESP_LOGI(TAG, "JoyC stick DOWN -> mic ON (down=%.2f)", down);
            } else if (mic_open && down < MIC_OFF_THRESH) {
                mic_open = false;
                if (stackchan_voice_is_listening()) {
                    stackchan_voice_toggle_listening();
                }
                ESP_LOGI(TAG, "JoyC stick returned -> mic OFF (down=%.2f)", down);
            }

            /* SIDE BUTTON -> take a photo and send it to OpenClaw to analyze.
             *   - "pressed" = button register differs from its boot-detected
             *     resting value (polarity never hardcoded).
             *   - Fire once on the released->pressed edge, 300 ms debounce.
             *   - Independent of stick position (mic is on the stick now). */
            int btn = joyc_read8(REG_BUTTON);
            if (btn >= 0) {
                bool pressed = (btn != s_btn_rest);
                if (pressed && !prev_pressed) {
                    int64_t now_us = esp_timer_get_time();
                    if (now_us - last_click_us > 300000) {
                        last_click_us = now_us;
                        bool started = stackchan_camera_capture_async(NULL);
                        ESP_LOGI(TAG, "JoyC SIDE BUTTON -> camera capture (started=%d)",
                                 (int)started);
                    }
                }
                prev_pressed = pressed;
            }
        }

        vTaskDelay(period);
    }
}

esp_err_t stackchan_joyc_start(void)
{
    /* Ensure the external I2C bus (Port A) is up. M5.begin() sets the port and
     * begins it when external RTC/IMU probing is enabled; begin() again is a
     * cheap no-op / re-init if needed. */
    if (!M5.Ex_I2C.isEnabled()) {
        M5.Ex_I2C.begin();
    }

    joyc_scan_bus();

    if (!joyc_probe()) {
        ESP_LOGW(TAG,
                 "Mini JoyC not found on Ex_I2C (0x%02X/0x%02X). "
                 "Plug it into red Port A and reboot to enable stick control.",
                 JOYC_ADDR, JOYC_ADDR_ALT);
        return ESP_OK; /* non-fatal: idle look-around keeps running */
    }

    int fw = joyc_read8(REG_FW_VER);
    ESP_LOGI(TAG, "Mini JoyC detected at 0x%02X (fw=%d)", s_addr, fw);

    joyc_capture_center();

    BaseType_t ok = xTaskCreatePinnedToCore(
        joyc_task, "joyc_ctl", 3072, NULL, 4, NULL, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create JoyC task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
