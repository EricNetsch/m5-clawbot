/*
 * OpenClaw StackChan remote — minimal custom firmware for M5StickC Plus + Hat Mini JoyC.
 *
 * Single black low-brightness screen with the OpenClaw avatar + a tiny mode label.
 * BtnA (front M5 button) cycles mode: OPERATE -> MOVE -> TWEAK.
 * Hardcoded ESP-NOW Wi-Fi channel 7, broadcast target-id 0.
 * Aggressive idle after 60s of no input (screen off + radio off) to save battery;
 * wakes on any button / stick-click / stick movement (may take ~1s to re-arm radio).
 *
 * 8-byte payload (little-endian, compatible with the esp-openclaw-node receiver):
 *   [0]   target-id (0 = broadcast)
 *   [1-2] yaw   int16  (+right .. -left, full-scale +/-1280)   <- stick X
 *   [3-4] pitch int16  (0..900, center ~450)                   <- stick Y
 *   [5]   mode   uint8 (0=OPERATE, 1=MOVE, 2=TWEAK)
 *   [6]   buttons bitfield: bit0 = stick-click, bit1 = BtnB
 *   [7]   seq   uint8  (rolling counter; liveness / activity hint)
 */
#include "M5Unified.h"

extern "C" {
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/i2c.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "i2c_bus.h"   /* espressif/i2c_bus: factory-proven JoyC read path */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_now_init.h"
}

#include "avatar_img.h"

using namespace m5;

static const char *TAG = "oc_remote";

/* -------------------------------------------------------------------- */
/* JoyC (M5 Hat Mini JoyC, I2C addr 0x54) via the ESP-IDF *hardware* I2C driver.
 *
 * ROOT CAUSE of the in-loop ok=0 (verified against factory + M5Unified source):
 * the old code read the JoyC through M5.Ex_I2C (the m5gfx *software* I2C engine).
 * On this StickC Plus, repeated in-loop Ex_I2C.readRegister() calls NAK/fail
 * even though a standalone init read works. The prime "M5.update() re-drives
 * GPIO0 every loop" hypothesis is FALSE: the CH552 push-pull-HIGH countermeasure
 * (M5Unified.cpp:1792) runs ONCE in M5::_begin(); StickCPlus M5.update() reads
 * only GPIO37/GPIO39 for BtnA/BtnB (M5Unified.cpp:2771) and never touches GPIO0.
 *
 * FIX: do EXACTLY what the M5 factory firmware does
 * (stackchan-remote .../joystick/joystick_handle.c): use the ESP-IDF legacy
 * *hardware* I2C peripheral on I2C_NUM_0, SDA=G0 SCL=G26, INTERNAL pull-ups
 * enabled, 100 kHz. i2c_driver_install() routes G0/G26 through the I2C
 * peripheral as open-drain (overriding the begin-time GPIO0 push-pull drive) and
 * the hardware peripheral -- unlike bit-banged Ex_I2C -- is immune to whatever
 * m5gfx/display activity was breaking the software reads. The factory PROVES
 * this coexists with M5.begin()+M5.update() running every loop. Installed ONCE
 * after M5.begin(); reads are plain hardware transactions each loop thereafter. */
#define JOYC_ADDR 0x54
#define JOYC_SDA  0            /* HAT header G0 = SDA */
#define JOYC_SCL  26           /* HAT header G26 = SCL */
#define JOYC_FREQ 100000
#define JOYC_PORT I2C_NUM_0    /* external bus; In_I2C(IMU/AXP)=NUM_1, no conflict */

/* ------------------------------------------------------------------ *
 * ROOT CAUSE (proven by simultaneous dual-serial 2026-08-16): the raw
 * i2c_master_write_read_device() path returned ESP_FAIL on 100% of reads
 * (remote logged ok=0 with x/y stuck at 2180/1960 while the stick moved),
 * so every ESP-NOW frame carried dead-center data and the robot head never
 * moved -- even though frames arrived perfectly (robot RAW-RX len=28 ch=7).
 *
 * FIX: use the espressif/i2c_bus component EXACTLY like the factory remote
 * (~/stackchan-remote/.../joystick/joystick_handle.c) and do the read in a
 * DEDICATED FreeRTOS task, OFF the thread that calls M5.update(). This is the
 * proven-working combination on this hardware (see main/idf_component.yml).
 * ------------------------------------------------------------------ */
static i2c_bus_handle_t        s_joyc_bus = NULL;
static i2c_bus_device_handle_t s_joyc_dev = NULL;

/* Latest JoyC sample, published by joyc_task and consumed by the main loop.
 * Defaults are the calibrated center (JX_CENTER/JY_CENTER, defined below). */
static volatile uint16_t s_jx     = 2180;
static volatile uint16_t s_jy     = 1960;
static volatile bool     s_jclick = false;
static volatile bool     s_joyc_ok = false;

/* #5-B: remote-LOCAL TWEAK tracking (display only; NO robot round-trip, NO
 * protocol change). Mirrors the robot's TWEAK ramp (rate 2.5/tick, Y(pitch)->
 * volume, X(yaw)->brightness, deadzone 0.12, same 0-255 / 8-255 clamps) so the
 * on-screen % follows what the robot is doing. Seeded at 50% (the remote can't
 * know the robot's actual value without a round-trip). */
static float s_tw_vol     = 128.0f;   /* 0..255   */
static float s_tw_bri     = 128.0f;   /* 8..255   */
static int   s_tw_vol_pct = 50;
static int   s_tw_bri_pct = 50;

static bool joyc_read(uint16_t *x, uint16_t *y, bool *click);

/* Install the ESP-IDF legacy *hardware* I2C master on JOYC_PORT (NUM_0), SDA=G0
 * SCL=G26, internal pull-ups on, 100 kHz -- byte-identical to the factory
 * joystick_i2c_init(). i2c_driver_install() reconfigures G0/G26 to the I2C
 * peripheral (open-drain), overriding the M5.begin() GPIO0 push-pull drive.
 * Call ONCE, after M5.begin(). */
static void joyc_init(void)
{
    /* FACTORY-EXACT joystick_i2c_init() verbatim (~/stackchan-remote/.../
     * joystick/joystick_handle.c): conf -> i2c_bus_create -> scan -> device_create.
     * NOTHING touches/drives G0/G26 before this -- no bus_recover, no PINCHK, no
     * gpio reclaim. This is the entire init. */
    i2c_config_t conf;
    memset(&conf, 0, sizeof(conf));
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = JOYC_SDA;
    conf.scl_io_num       = JOYC_SCL;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = JOYC_FREQ;
    conf.clk_flags        = 0;

    s_joyc_bus = i2c_bus_create(JOYC_PORT, &conf);
    if (!s_joyc_bus) {
        ESP_LOGE(TAG, "JoyC i2c_bus_create FAILED (port=%d SDA=%d SCL=%d)",
                 (int)JOYC_PORT, JOYC_SDA, JOYC_SCL);
        return;
    }

    /* Factory logs the scan; keep it so we can see whether 0x54 ACKs. (On a
     * dead bus this times out across all addresses ~25s; on a live bus it is
     * fast.) */
    uint8_t found[128];
    memset(found, 0, sizeof(found));
    uint8_t n = i2c_bus_scan(s_joyc_bus, found, sizeof(found));
    ESP_LOGW(TAG, "I2CSCAN port0: %d device(s) ACKed", (int)n);
    for (size_t i = 0; i < sizeof(found); i++) {
        if (found[i]) ESP_LOGW(TAG, "I2CSCAN: found 0x%02X", found[i]);
    }

    s_joyc_dev = i2c_bus_device_create(s_joyc_bus, JOYC_ADDR, 0);
    ESP_LOGW(TAG, "JoyC dev_create 0x%02X -> dev=%p (scan=%d)",
             JOYC_ADDR, (void *)s_joyc_dev, (int)n);

    /* FACTORY-EXACT: joystick_i2c_init() ends HERE -- create+scan+device_create,
     * NO read. Factory never touches the JoyC during init; the FIRST read happens
     * later, in the read task that is created only AFTER wifi_espnow_init(). Doing
     * an immediate probe read here (during boot, before wifi is up) is one of the
     * two deltas that wedged the STM32 -- removed. */
}

/* Read X (reg 0x00, 2B LE) and Y (reg 0x02, 2B LE) ONLY -- axes-only, no 0x30
 * via the espressif/i2c_bus component -- byte-identical registers + separate-read
 * pattern + 20ms inter-axis settle as the factory joystick_read_xy(). */
static bool joyc_read(uint16_t *x, uint16_t *y, bool *click)
{
    if (!s_joyc_dev) { s_joyc_ok = false; return false; }

    uint8_t d[4] = {0};
    esp_err_t rx = i2c_bus_read_bytes(s_joyc_dev, 0x00, 2, &d[0]);
    /* Factory waits between the two axis reads for data stability. */
    vTaskDelay(pdMS_TO_TICKS(20));
    esp_err_t ry = i2c_bus_read_bytes(s_joyc_dev, 0x02, 2, &d[2]);

    bool ok = (rx == ESP_OK && ry == ESP_OK);
    if (ok) {
        if (x) *x = (uint16_t)((d[1] << 8) | d[0]);
        if (y) *y = (uint16_t)((d[3] << 8) | d[2]);
        /* AXES-ONLY, PERMANENT: NEVER read the 0x30 button register -- that poll
         * is what wedged/bricked the STM32. click stays false here; OPERATE-down
         * is derived purely from the Y axis (0x02 -> pitch) in the main loop. */
        if (click) *click = false;
    } else {
        static int64_t last_err_us = 0;
        int64_t nowu = esp_timer_get_time();
        if (nowu - last_err_us > 1000000) {
            last_err_us = nowu;
            ESP_LOGW(TAG, "JoyC read fail rx=%s ry=%s",
                     esp_err_to_name(rx), esp_err_to_name(ry));
        }
    }
    s_joyc_ok = ok;
    return ok;
}

static inline bool joyc_ok(void) { return s_joyc_ok; }

/* Dedicated JoyC read task -- runs OFF the app_main thread that calls
 * M5.update(). This is the factory architecture (reads in handle_running_screen
 * task while app_main separately drives M5.update). Publishes the latest sample
 * to s_jx/s_jy/s_jclick/s_joyc_ok at ~25 Hz. */
static void joyc_task(void *arg)
{
    (void)arg;
    /* No pre-delay needed: this task is now created AFTER wifi_espnow_init()
     * (factory ordering), so the FIRST read already happens well past the
     * volatile WiFi/ESP-NOW bring-up window that was stranding the STM32. */
    for (;;) {
        uint16_t x = s_jx, y = s_jy; bool c = false;
        /* On failure: DO NOT recover/re-init/toggle lines -- just skip and retry
         * next cycle (joyc_read already logs, never touches the bus manually). */
        if (joyc_read(&x, &y, &c)) {
            s_jx = x; s_jy = y; s_jclick = c;
        }
        /* TRANSPLANT: factory handle_running_screen cadence = 30ms (the mode the
         * proven-holding bin ran in while it held). */
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

/* ---- Hardcoded link config ---- */
static const uint8_t ESPNOW_CHANNEL = 7;   /* must match the robot's Wi-Fi channel */
static const uint8_t TARGET_ID      = 0;   /* 0 = broadcast to any receiver */

/* JoyC 16-bit ADC calibration (VERBATIM from M5 factory ui_running_screen.h). */
#define JX_CENTER 2180
#define JY_CENTER 1960
#define JX_MIN    630
#define JX_MAX    3730
#define JY_MIN    310
#define JY_MAX    3460
#define JDEAD     300

#define jmap(x, in_min, in_max, out_min, out_max) \
    (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

/* ---- Modes ---- */
enum { MODE_OPERATE = 0, MODE_MOVE = 1, MODE_TWEAK = 2, MODE_COUNT = 3 };
static const char *MODE_NAME[MODE_COUNT] = {"OPERATE", "MOVE", "TWEAK"};
/* Accent color per mode (RGB565). */
static uint16_t mode_color(uint8_t m)
{
    switch (m) {
        case MODE_OPERATE: return 0x07FF; /* cyan  */
        case MODE_MOVE:    return 0x07E0; /* green */
        default:           return 0xFD20; /* amber */
    }
}

/* ---- Idle tuning ---- */
static const int64_t IDLE_AFTER_US = 60LL * 1000000LL; /* 60s of no input -> idle */
static const uint8_t SCREEN_BRIGHT = 70;               /* prior comfortable level (Eric preferred this over 50) */

/* -------------------------------------------------------------------- */
/* Radio power control: fully drop the radio in idle, bring it back on wake. */
static void radio_start(void)
{
    esp_wifi_start();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    espnow_config_t cfg = ESPNOW_INIT_CONFIG_DEFAULT();
    cfg.forward_enable         = false;
    cfg.forward_switch_channel = false;
    cfg.send_retry_num         = 5;
    cfg.receive_enable.forward = false;
    cfg.receive_enable.data    = false;
    espnow_init(&cfg);
}
static void radio_stop(void)
{
    espnow_deinit();
    esp_wifi_stop();
}

/* -------------------------------------------------------------------- */
/* #5-B: draw the live TWEAK brightness/volume %s in the band above the avatar
 * (avatar is pinned at y=138). Clears only that band so the avatar/mode label
 * don't flicker. */
static void draw_tweak_pct(void)
{
    auto &d = M5.Display;
    d.fillRect(0, 40, d.width(), 95, 0x0000);   /* band y=40..135, above avatar */
    d.setTextColor(0xFFFF, 0x0000);
    d.setTextSize(2);
    char line[20];
    snprintf(line, sizeof(line), "Bri:%d%%", s_tw_bri_pct);
    d.drawCenterString(line, d.width() / 2, 55);
    snprintf(line, sizeof(line), "Vol:%d%%", s_tw_vol_pct);
    d.drawCenterString(line, d.width() / 2, 90);
}

/* -------------------------------------------------------------------- */
static void draw_screen(uint8_t mode, int bat)
{
    auto &d = M5.Display;
    /* jem-idle face is 135x102 (fit-to-width, NOT stretched). PIN it flush to the
     * BOTTOM edge of the black 135x240 screen: y = 240 - 102 = 138 (fully visible,
     * bottom-anchored). Was y=69 (vertically centered). */
    d.fillScreen(0x0000);
    d.drawJpg(AVATAR_JPG, AVATAR_JPG_LEN, 0, 138);
    /* Tiny mode label, top-center. */
    d.setTextColor(mode_color(mode), 0x0000);
    d.setTextSize(2);
    d.drawCenterString(MODE_NAME[mode], d.width() / 2, 4);
    /* Subtle battery %, top-right. */
    if (bat >= 0) {
        char b[12];
        snprintf(b, sizeof(b), "%d", bat);
        d.setTextColor(0x7BEF, 0x0000); /* dim gray */
        d.setTextSize(1);
        d.drawRightString(b, d.width() - 3, 6);
    }
    /* #5-B: in TWEAK, show the live brightness/volume %s above the avatar. */
    if (mode == MODE_TWEAK) draw_tweak_pct();
}

/* -------------------------------------------------------------------- */
extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
    }

    /* ================= FACTORY-EXACT init (2026-08-16) =================
     * Matches ~/stackchan-remote StackChan-RemoteControl-ESPNow.cpp +
     * joystick_handle.c EXACTLY. We ripped out EVERY addition that differed
     * from the proven-good factory -- output_power/internal_spk/internal_mic
     * mcfg, M5.Power.setExtOutput + the EXTEN power-cycle toggle, gpio reclaim,
     * joyc_bus_recover, PINCHK, rtc/gpio hold-dis, and M5.Imu.init -- because
     * those were the ONLY deltas from factory and were what left SCL(G26)
     * wedged. NOTHING here touches or drives G0/G26 before i2c_bus_create. */
    /* BYTE-IDENTICAL to the factory remote init (StackChan-RemoteControl-
     * ESPNow.cpp): plain M5.begin() with ALL M5 defaults. M5.begin() internally
     * runs Power.begin() then Power.setExtOutput(cfg.output_power=true) ->
     * Axp192.setEXTEN(true) = HAT-header 5V ON (M5Unified.cpp:1743-1744). So the
     * rail is enabled and LEFT ON by the default state, exactly like factory --
     * NO mcfg overrides, NO explicit setExtOutput, NO toggles, NO gpio reclaim. */
    M5.begin();
    M5.Power.begin();
    M5.Display.setRotation(0);          /* 135x240 portrait */
    M5.Display.setBrightness(SCREEN_BRIGHT);
    M5.Imu.init(&M5.In_I2C);            /* factory does this (In_I2C = port 1) */

    /* MEASURED proof of the ACTUAL HAT-5V rail state -- real AXP192 registers,
     * not just our intended call -- BEFORE the I2C scan. EXTEN = reg0x12 bit6.
     * VBUS present (USB) also feeds the 5V bus. */
    {
        auto &ax = M5.Power.Axp192;
        ESP_LOGW(TAG, "AXP192 rail: EXTEN=%d reg0x12=0x%02X reg0x10=0x%02X "
                      "VBUS=%d(%.2fV) ACIN=%d BAT=%.2fV | boot SCL(G%d)=%d SDA(G%d)=%d",
                 (int)ax.getEXTEN(), ax.readRegister8(0x12), ax.readRegister8(0x10),
                 (int)ax.isVBUS(), ax.getVBUSVoltage(),
                 (int)ax.isACIN(), ax.getBatteryVoltage(),
                 JOYC_SCL, gpio_get_level((gpio_num_t)JOYC_SCL),
                 JOYC_SDA, gpio_get_level((gpio_num_t)JOYC_SDA));
    }

    joyc_init();   /* factory-exact: i2c_bus_create -> scan(log) -> device_create(0x54), NO read */

    /* Wi-Fi + ESP-NOW once (netif + event loop live for the whole run).
     * FACTORY ORDERING: bring WiFi/ESP-NOW fully up BEFORE any JoyC reads start.
     * Reads that overlapped WiFi radio-calibration (long ISR-off windows) were
     * timing-out mid-clock-stretch and stranding the STM32 -> wedge. */
    wifi_espnow_init(ESPNOW_CHANNEL);

    /* Only NOW start the dedicated JoyC reader -- exactly like factory, which
     * xTaskCreate()s its read tasks AFTER wifi_espnow_init(). Unpinned, prio 5
     * to match factory (handle_setup/running_screen). */
    xTaskCreate(joyc_task, "joyc", 8192, NULL, 5, NULL);
    {
        uint8_t ch; wifi_second_chan_t sec;
        esp_wifi_get_channel(&ch, &sec);
        ESP_LOGI(TAG, "ESP-NOW up on channel %d (target-id %d)", ch, TARGET_ID);
    }

    uint8_t  mode      = MODE_OPERATE;
    int      bat       = M5.Power.getBatteryLevel();
    draw_screen(mode, bat);

    uint8_t  pkt[8]    = {0};
    uint8_t  seq       = 0;
    bool     idle      = false;
    bool     prev_click = false;
    int64_t  last_activity_us = esp_timer_get_time();
    int64_t  last_bat_us      = 0;

    while (1) {
        M5.update();
        int64_t now = esp_timer_get_time();

        /* ---- Inputs ---- */
        bool btnA = M5.BtnA.wasPressed();  /* front M5 button: cycle mode */
        bool btnB = M5.BtnB.isPressed();   /* side button: transmitted bit */
        /* Latest JoyC sample published by the dedicated joyc_task (factory
         * architecture: I2C reads happen OFF this M5.update() thread). */
        uint16_t rx = s_jx, ry = s_jy;
        bool click = s_jclick;

        /* Deadzone-corrected magnitude for activity detection. */
        int dx = (int)rx - JX_CENTER; if (dx < 0) dx = -dx;
        int dy = (int)ry - JY_CENTER; if (dy < 0) dy = -dy;
        bool stick_moved = (dx > JDEAD) || (dy > JDEAD);
        bool click_edge  = (click && !prev_click);
        prev_click = click;

        bool activity = btnA || click || click_edge || btnB || stick_moved;

        /* ---- Idle / wake ---- */
        if (idle) {
            if (activity) {
                idle = false;
                radio_start();
                M5.Display.wakeup();
                M5.Display.setBrightness(SCREEN_BRIGHT);
                draw_screen(mode, bat);
                last_activity_us = now;
                ESP_LOGI(TAG, "wake (activity)");
            } else {
                vTaskDelay(pdMS_TO_TICKS(200)); /* slow poll while asleep */
                continue;
            }
        }
        if (activity) last_activity_us = now;

        /* ---- Mode switch (BtnA) ---- */
        if (btnA) {
            mode = (mode + 1) % MODE_COUNT;
            draw_screen(mode, bat);
            ESP_LOGI(TAG, "mode -> %s", MODE_NAME[mode]);
        }

        /* ---- Refresh battery label occasionally ---- */
        if (now - last_bat_us > 5000000LL) {
            last_bat_us = now;
            int nb = M5.Power.getBatteryLevel();
            if (nb != bat) { bat = nb; draw_screen(mode, bat); }
        }

        /* ---- Build + send payload ---- */
        uint16_t jx = rx, jy = ry;
        if ((int)jx < JX_CENTER + JDEAD && (int)jx > JX_CENTER - JDEAD) jx = JX_CENTER;
        if ((int)jy < JY_CENTER + JDEAD && (int)jy > JY_CENTER - JDEAD) jy = JY_CENTER;
        int16_t yaw   = (int16_t)jmap((int)jx, JX_MIN, JX_MAX, 1280, -1280);
        int16_t pitch = (int16_t)jmap((int)jy, JY_MIN, JY_MAX, 0, 900);
        if (yaw > 1280) { yaw = 1280; }
        if (yaw < -1280) { yaw = -1280; }
        if (pitch > 900) { pitch = 900; }
        if (pitch < 0) { pitch = 0; }

        /* OPERATE-down (AXES-ONLY): map the DOWN direction of the Y axis we
         * ALREADY read (0x02 -> pitch) to the OPERATE-down action. NO 0x30, NO
         * button read anywhere. The robot toggles the mic on buttons bit0 (rising
         * edge); we set bit0 when the stick is pushed DOWN past threshold in
         * OPERATE. DOWN = low pitch (robot treats high pitch = UP), pitch <= 315
         * (symmetric to the robot's UP barge-in at ny>0.30; past ~471 center +
         * deadzone so it only fires on a deliberate down-push). */
        bool stick_down = (mode == MODE_OPERATE) && (pitch <= 315);
        uint8_t buttons = (stick_down ? 0x01 : 0x00) | (btnB ? 0x02 : 0x00);

        /* #5-B: TWEAK -- track brightness/volume % LOCALLY (display only). Mirror
         * the robot's ramp EXACTLY: ny=(pitch-450)/450 -> volume, nx=yaw/1280 ->
         * brightness, rate 2.5/tick, deadzone 0.12, clamps 0-255 / 8-255. The
         * remote sends one packet per loop and the robot ramps once per packet,
         * so the % tracks the robot's change (offset only by the 50% seed). */
        if (mode == MODE_TWEAK) {
            float ny = (float)(pitch - 450) / 450.0f;   /* +up / -down */
            float nx = (float)yaw / 1280.0f;
            if (ny >  1.0f) ny =  1.0f; else if (ny < -1.0f) ny = -1.0f;
            if (nx >  1.0f) nx =  1.0f; else if (nx < -1.0f) nx = -1.0f;
            if (ny > 0.12f || ny < -0.12f) {
                s_tw_vol += ny * 2.5f;
                if (s_tw_vol < 0.0f) s_tw_vol = 0.0f; else if (s_tw_vol > 255.0f) s_tw_vol = 255.0f;
            }
            if (nx > 0.12f || nx < -0.12f) {
                s_tw_bri += nx * 2.5f;
                if (s_tw_bri < 8.0f) s_tw_bri = 8.0f; else if (s_tw_bri > 255.0f) s_tw_bri = 255.0f;
            }
            int vp = (int)(s_tw_vol / 255.0f * 100.0f + 0.5f);
            int bp = (int)(s_tw_bri / 255.0f * 100.0f + 0.5f);
            if (vp != s_tw_vol_pct || bp != s_tw_bri_pct) {
                s_tw_vol_pct = vp; s_tw_bri_pct = bp;
                draw_tweak_pct();
            }
        }

        pkt[0] = TARGET_ID;
        memcpy(&pkt[1], &yaw, 2);
        memcpy(&pkt[3], &pitch, 2);
        pkt[5] = mode;
        pkt[6] = buttons;
        pkt[7] = seq++;
        espnow_send_data(pkt, sizeof(pkt));

        /* Throttled telemetry so stick movement is verifiable over serial. */
        static int64_t last_dbg_us = 0;
        if (now - last_dbg_us > 500000LL) {
            last_dbg_us = now;
            /* EXPERIMENT A: NO gpio_get_level() on G26/G0 -- those pins are owned
             * by the I2C peripheral; reading them is a delta from factory. Report
             * ok + raw only (AXP reads are on In_I2C port1, unrelated to JoyC). */
            ESP_LOGI(TAG, "%s raw x=%u y=%u ok=%d | EXTEN=%d VBUS=%d(%.2fV) BAT=%.2fV",
                     MODE_NAME[mode], (unsigned)rx, (unsigned)ry, (int)joyc_ok(),
                     (int)M5.Power.Axp192.getEXTEN(),
                     (int)M5.Power.Axp192.isVBUS(), M5.Power.Axp192.getVBUSVoltage(),
                     M5.Power.Axp192.getBatteryVoltage());
        }

        /* ---- Enter idle after inactivity ---- */
        if (now - last_activity_us > IDLE_AFTER_US) {
            idle = true;
            radio_stop();
            M5.Display.setBrightness(0);
            M5.Display.sleep();
            ESP_LOGI(TAG, "-> idle (radio off, screen off)");
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); /* ~50 Hz while awake -- snappy button/toggle */
    }
}
