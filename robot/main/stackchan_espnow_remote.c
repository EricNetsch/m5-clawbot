/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP-NOW receiver for the M5 StackChan battery-powered joystick remote.
 *
 * ==========================================================================
 *  PROTOCOL (reverse-engineered from the M5 factory firmware source)
 *  Repo: github.com/m5stack/StackChan
 *   - Robot RX side: firmware/main/apps/app_espnow_ctrl/app_espnow_ctrl.cpp
 *   - Remote TX side: remote/code/main/joystick/joystick_handle.c
 *
 *  Both ends use the espressif/esp-now managed component (NOT raw esp_now).
 *  That component prepends its own application header (espnow_data_t) to the
 *  8-byte joystick payload before handing it to esp_now_send(). So a *raw* IDF
 *  esp_now recv callback receives:
 *
 *      [ espnow_data_t header ][ 8-byte joystick payload ]
 *
 *  We reproduce the component's exact on-wire header struct below so the
 *  compiler computes the identical byte offset, then read the payload that
 *  follows it. (This avoids pulling the whole esp-now component into our
 *  firmware and re-initialising Wi-Fi, which the node already owns.)
 *
 *  8-byte joystick payload (little-endian). UPDATED for the OpenClaw custom
 *  remote (~/stackchan-tx). Yaw/pitch encoding is byte-identical to the factory
 *  so this receiver stays compatible; bytes 5-7 are repurposed:
 *      [0]    target-id (uint8)   0 = broadcast, else must match receiver id
 *      [1..2] yaw-angle  (int16)  -1280 .. 1280   (stick X, +right/-left)
 *      [3..4] pitch-angle(int16)   0 .. 900       (stick Y, center ~450)
 *      [5]    mode      (uint8)   0=OPERATE, 1=MOVE, 2=TWEAK
 *      [6]    buttons   (uint8)   bit0 = stick-click, bit1 = BtnB
 *      [7]    seq       (uint8)   rolling counter (liveness hint)
 *
 *  Mode routing:
 *    OPERATE : stick gestures. stick-DOWN hold = mic push-to-talk (the push-to-talk
 *              that BtnB used to drive); stick-UP = barge-in (abort playback);
 *              stick-LEFT = snap a photo. No servo motion.
 *    MOVE    : stick drives the head yaw/pitch via the servos (no gestures).
 *    TWEAK   : stick Y ramps robot speaker VOLUME, stick X ramps LCD BRIGHTNESS.
 *
 *  note_activity(): called on EVERY received frame so the remote keeps the
 *  robot awake / wakes it from light idle (Tier1 keeps the radio hot).
 *
 *  Channel / broadcast: the remote broadcasts (FF:FF:FF:FF:FF:FF) on a fixed
 *  Wi-Fi channel it is configured for (default 1, selectable 1..13 on the
 *  remote's advanced setup screen). Our radio is a station locked to the AP's
 *  channel, so reception only works when the remote's channel matches the
 *  robot's current Wi-Fi channel. We log our channel at startup so it can be
 *  matched; we never change our own channel (that would drop the node link).
 * ==========================================================================
 */

#include "stackchan_espnow_remote.h"
#include "stackchan_servo.h"
#include "stackchan_voice.h"
#include "stackchan_camera.h"
#include "stackchan_idle.h"   /* note_activity: keep-alive / wake from light idle */
#include "stackchan_body.h"   /* display brightness + speaker volume (TWEAK mode) */

#include <math.h>
#include <string.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "stackchan_espnow";

/* ---------------------------------------------------------------------------
 * Exact on-wire header of the espressif/esp-now component (byte-identical to
 * esp-now <= v2.5.x). Copied verbatim so sizeof() gives the same layout the
 * remote used when it built the frame. DO NOT "clean up" these bitfields.
 * ------------------------------------------------------------------------- */
typedef struct espnow_frame_head_s {
    uint16_t magic;
    uint8_t channel              : 4;
    bool filter_adjacent_channel : 1;
    bool filter_weak_signal      : 1;
    bool security                : 1;
    uint16_t                     : 4;
    bool broadcast              : 1;
    bool group                  : 1;
    bool ack                    : 1;
    uint16_t retransmit_count   : 5;
    uint8_t forward_ttl         : 5;
    int8_t forward_rssi         : 8;
} __attribute__((packed)) espnow_frame_head_t;

typedef struct {
    uint8_t type    : 4;
    uint8_t version : 2;
    uint8_t         : 2;
    uint8_t size;
    espnow_frame_head_t frame_head;
    uint8_t dest_addr[6];
    uint8_t src_addr[6];
    uint8_t payload[0];
} __attribute__((packed)) espnow_app_hdr_t;

#define ESPNOW_JOY_PAYLOAD_LEN 8

/* ---------------------------------------------------------------------------
 * Control tuning. These are the knobs Eric will most likely want to touch.
 * ------------------------------------------------------------------------- */
/* Wire full-scale values from the remote firmware. */
static const float YAW_WIRE_FULL   = 1280.0f; /* yaw wire range is -1280..+1280 */
static const float PITCH_WIRE_CTR  = 450.0f;  /* pitch wire range 0..900, center ~450 */
static const float PITCH_WIRE_SPAN = 450.0f;  /* center -> full deflection */

static const int   YAW_RANGE_DEG   = 125;  /* full-scale yaw in degrees (factory range) */
static const int   PITCH_RANGE_DEG = 80;   /* full-scale pitch in degrees (factory range) */
static const float DEADZONE        = 0.12f;/* ~12% resting deadzone (normalized magnitude) */
static const int   MOVE_SPEED      = 900;  /* 0..1000 servo move speed */
static const int   SEND_THRESH_DEG = 2;    /* only resend when target moved this much */
static const int64_t IDLE_RESUME_MS = 5000; /* resume ambient look-around after this much inactivity */

/* ---- Mic-on-stick-DOWN tuning ----
 * Pull the stick DOWN to open the mic; let it return toward center to close it.
 * Intentionally sloppy on BOTH ends (wide hysteresis) so it triggers over a
 * broad range of down-deflection and closes over a broad center zone.
 *   ON  : down-deflection exceeds MIC_ON_THRESH (small -> easy to trigger).
 *   OFF : down-deflection falls back under MIC_OFF_THRESH (wide center zone). */
static const float MIC_ON_THRESH  = 0.30f; /* ~30% down opens the mic        */
static const float MIC_OFF_THRESH = 0.12f; /* back under ~12% down closes it */

/* ---- Camera-on-stick-LEFT tuning ----
 * Deliberately DEEP + horizontal-dominant so it is hard to trigger by accident
 * (a casual down/up nudge for mic/barge-in won't fire it).
 *   ON  : left-deflection past CAM_ON_THRESH AND mostly horizontal (|x|>|y|).
 *   OFF : re-arms once left-deflection falls back under CAM_OFF_THRESH. */
static const float CAM_ON_THRESH  = 0.55f; /* need ~55% left to snap a photo   */
static const float CAM_OFF_THRESH = 0.30f; /* re-arm below ~30% left           */
/* Separate the left-camera flick from the down-mic pull: require the vertical
 * component stay modest so a diagonal down-left goes to mic, not camera. */
static const float CAM_MAX_VERT   = 0.55f; /* |ny| must be under this to snap  */

/* ---- Remote modes (payload byte 5), must match ~/stackchan-tx main.cpp ---- */
enum { REMOTE_MODE_OPERATE = 0, REMOTE_MODE_MOVE = 1, REMOTE_MODE_TWEAK = 2 };
/* ---- Buttons bitfield (payload byte 6) ---- */
#define BTN_STICK_CLICK 0x01
#define BTN_SIDE_B      0x02

/* ---- TWEAK-mode ramp tuning ----
 * Stick Y ramps speaker volume, stick X ramps LCD brightness. We RAMP (rather
 * than absolute-map) so entering TWEAK does not snap the value to the stick's
 * resting position: hold up/down (or left/right) to slew, release to hold. Rate
 * is per-packet (~30 Hz), scaled by deflection; a full-deflection sweep of the
 * whole 0-255 range takes roughly 1.5 s. */
static const float TWEAK_VOL_RATE = 2.5f;  /* volume units per packet at full Y (was 7.0 -- slower/finer TWEAK) */
static const float TWEAK_BRI_RATE = 2.5f;  /* brightness units per packet at full X (was 7.0 -- slower/finer TWEAK) */
static const float TWEAK_MIN_BRI  = 8.0f;  /* keep the panel visible (backlight floor) */


/* Set while the remote holds control; the worker resumes idle once the stick
 * has been quiet (centered or remote off) for IDLE_RESUME_MS. */
static volatile int64_t s_last_active_ms = 0;
static volatile bool    s_engaged = false;

static inline int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

/* Axis sign conventions. Flip a single line if the stick feels inverted.
 *   X: stick right -> yaw right (positive, per servo API pos=right). */
static const float X_SIGN __attribute__((unused)) = +1.0f;
/*   Y: stick up -> head up (positive). Flip to -1.0f if reversed on the bench. */
static const float Y_SIGN = +1.0f;

/* Set to 0 to require target_id match; 1 = accept any id (re-pairing friendly).
 * We don't know which receiver-id Eric's remote is set to, so default is to
 * accept every id and just log it. */
static const int ACCEPT_ANY_ID = 1;
static const uint8_t OUR_RECEIVER_ID = 1; /* factory default receiver id */

/* ---------------------------------------------------------------------------
 * Internal plumbing.
 * ------------------------------------------------------------------------- */
typedef struct {
    uint8_t mac[6];
    uint8_t data[ESPNOW_JOY_PAYLOAD_LEN];
    int total_len; /* full raw frame length (header + payload) for logging */
} espnow_evt_t;

static QueueHandle_t s_queue = NULL;

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Raw ESP-NOW recv callback (runs in the Wi-Fi task context, not ISR).
 * Keep it lightweight: log for capture, strip the component header, and push
 * the 8-byte joystick payload onto the queue for the worker task. */
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    static uint32_t count = 0;
    static int64_t s_last_rxlog_us = 0;

    const uint8_t *mac = (info && info->src_addr) ? info->src_addr : (const uint8_t *)"\0\0\0\0\0\0";
    count++;

    /* DIAG: log EVERY raw esp_now frame that reaches us (rate-limited to ~4/s),
     * BEFORE any length filtering, so a channel/peer break is unambiguous:
     * if this never prints while the remote claims to send, no frames arrive. */
    {
        int64_t nowu = esp_timer_get_time();
        if (nowu - s_last_rxlog_us > 250000) {
            s_last_rxlog_us = nowu;
            uint8_t ch = 0; wifi_second_chan_t sc = WIFI_SECOND_CHAN_NONE;
            esp_wifi_get_channel(&ch, &sc);
            ESP_LOGW(TAG, "RAW-RX #%u len=%d ch=%d src=%02X:%02X:%02X:%02X:%02X:%02X",
                     (unsigned)count, len, (int)ch,
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        }
    }

    /* --- strip the esp-now component header, recover the joystick payload --- */
    const int hdr = (int)sizeof(espnow_app_hdr_t);
    if (len < hdr + ESPNOW_JOY_PAYLOAD_LEN) {
        return; /* not a StackChan-remote data frame (or too short) */
    }
    const uint8_t *payload = data + hdr;

    /* Any valid remote frame is activity: keep the robot awake and, if it is in
     * light idle (Tier1, radio still hot), wake it back to ACTIVE. Cheap + safe
     * from this Wi-Fi-task context. Done here so it counts even if the worker
     * queue is momentarily full. */
    stackchan_idle_note_activity();

    espnow_evt_t evt;
    memcpy(evt.mac, mac, 6);
    memcpy(evt.data, payload, ESPNOW_JOY_PAYLOAD_LEN);
    evt.total_len = len;

    if (s_queue) {
        (void)xQueueSend(s_queue, &evt, 0); /* drop if full; newest wins next time */
    }
}

static void espnow_apply_payload(const espnow_evt_t *evt)
{
    /* Persistent state across packets. */
    static uint8_t prev_mode  = 0xFF;  /* force a mode-enter log on first packet */
    static bool    prev_click = false; /* stick-click edge (OPERATE PTT)         */
    static bool    prev_btnB  = false; /* BtnB edge (global mic toggle)          */
    static bool    up_fired   = false; /* barge-in edge latch (stick-up)         */
    static bool    cam_fired  = false; /* camera edge latch (stick-left)         */
    static bool    mic_open   = false; /* mic latch driven by stick-click        */
    /* MOVE servo tracking */
    static bool    move_active = false;
    static int     last_yaw = 0, last_pitch = 0;
    /* TWEAK ramp state */
    static bool    tweak_seed = false;
    static float   vol_f = 128.0f, bri_f = 128.0f;
    static int     last_vol = -1, last_bri = -1;
    static int64_t dbg_last_us = 0;

    const uint8_t *d = evt->data;

    uint8_t target_id = d[0];
    if (!ACCEPT_ANY_ID && target_id != 0 && target_id != OUR_RECEIVER_ID) {
        return; /* addressed to a different receiver */
    }

    int16_t yaw_wire   = (int16_t)(d[1] | (d[2] << 8));
    int16_t pitch_wire = (int16_t)(d[3] | (d[4] << 8));
    uint8_t mode       = d[5];
    uint8_t buttons    = d[6];
    /* d[7] = rolling seq counter (liveness only) */

    float nx = X_SIGN * clampf((float)yaw_wire / YAW_WIRE_FULL, -1.0f, 1.0f);
    float ny = Y_SIGN * clampf(((float)pitch_wire - PITCH_WIRE_CTR) / PITCH_WIRE_SPAN, -1.0f, 1.0f);
    float mag  = sqrtf(nx * nx + ny * ny);
    float up   =  ny; /* positive when the stick is pushed UP   */
    float left =  nx; /* positive when pushed LEFT (sign as tuned for factory yaw) */
    bool  click = (buttons & BTN_STICK_CLICK) != 0;
    bool  btnB  = (buttons & BTN_SIDE_B) != 0;

    /* ---- BtnB -> global mic/PTT toggle (rising edge, ANY mode) ----
     * BtnB is the StickC side button (GPIO37/39), read via M5.BtnB and sent in
     * the packet -- it is INDEPENDENT of the JoyC I2C bus. This lets us verify
     * the full wireless chain (remote TX -> robot RX -> action) even when the
     * joystick is dead. */
    if (btnB && !prev_btnB) {
        bool want = !stackchan_voice_is_listening();
        stackchan_voice_toggle_listening();
        ESP_LOGI(TAG, "remote BtnB -> mic %s", want ? "ON" : "OFF");
    }
    prev_btnB = btnB;

    /* ---- Mode-change bookkeeping ---- */
    if (mode != prev_mode) {
        ESP_LOGI(TAG, "remote mode -> %s",
                 mode == REMOTE_MODE_OPERATE ? "OPERATE" :
                 mode == REMOTE_MODE_MOVE    ? "MOVE"    :
                 mode == REMOTE_MODE_TWEAK   ? "TWEAK"   : "?");
        if (prev_mode == REMOTE_MODE_MOVE) {
            /* leaving MOVE: hand the head back to ambient look-around */
            stackchan_servo_idle_set(true);
            move_active = false;
            last_yaw = last_pitch = 0;
        }
        if (prev_mode == REMOTE_MODE_OPERATE && mic_open) {
            /* leaving OPERATE with mic open: close it so it can't get stuck */
            mic_open = false;
            if (stackchan_voice_is_listening()) stackchan_voice_toggle_listening();
        }
        if (mode == REMOTE_MODE_TWEAK) {
            /* seed ramp accumulators from live values so nothing jumps */
            vol_f = (float)stackchan_get_volume();
            bri_f = (last_bri >= 0) ? (float)last_bri : 128.0f;
            tweak_seed = true;
        }
        up_fired = cam_fired = false;
        prev_click = click; /* suppress a spurious click edge across the switch */
        prev_mode = mode;
    }

    switch (mode) {
    case REMOTE_MODE_MOVE: {
        /* Stick drives the head yaw/pitch via the servos. */
        s_engaged = true;
        s_last_active_ms = now_ms();
        if (mag < DEADZONE) {
            move_active = false;
            last_yaw = last_pitch = 0;
        } else {
            int yaw_deg   = (int)lroundf(nx * YAW_RANGE_DEG);
            int pitch_deg = (int)lroundf(ny * PITCH_RANGE_DEG);
            if (!move_active || abs(yaw_deg - last_yaw) >= SEND_THRESH_DEG ||
                abs(pitch_deg - last_pitch) >= SEND_THRESH_DEG) {
                stackchan_servo_move(yaw_deg, pitch_deg, MOVE_SPEED);
                last_yaw = yaw_deg;
                last_pitch = pitch_deg;
                move_active = true;
            }
        }
        break;
    }
    case REMOTE_MODE_TWEAK: {
        /* Stick Y ramps speaker volume; stick X ramps LCD brightness. */
        if (!tweak_seed) { vol_f = (float)stackchan_get_volume(); tweak_seed = true; }
        if (fabsf(ny) > DEADZONE) {
            vol_f = clampf(vol_f + ny * TWEAK_VOL_RATE, 0.0f, 255.0f);
            int v = (int)lroundf(vol_f);
            if (v != last_vol) { stackchan_set_volume((uint8_t)v); last_vol = v; }
        }
        if (fabsf(nx) > DEADZONE) {
            bri_f = clampf(bri_f + nx * TWEAK_BRI_RATE, TWEAK_MIN_BRI, 255.0f);
            int b = (int)lroundf(bri_f);
            if (b != last_bri) { stackchan_display_brightness((uint8_t)b); last_bri = b; }
        }
        if (fabsf(nx) > DEADZONE || fabsf(ny) > DEADZONE) {
            int64_t nowu = esp_timer_get_time();
            if (nowu - dbg_last_us > 300000) {
                dbg_last_us = nowu;
                ESP_LOGI(TAG, "TWEAK vol=%d bri=%d (nx=%.2f ny=%.2f)", last_vol, last_bri, nx, ny);
            }
        }
        break;
    }
    case REMOTE_MODE_OPERATE:
    default: {
        /* ---- stick-DOWN -> PUSH-TO-TALK (momentary hold, NOT toggle) ----
         * The remote sends buttons bit0 (`click`) as a LEVEL: true while the
         * stick is held DOWN past threshold, false once it returns toward center.
         * Mic follows that level: HOLD DOWN = mic open (talk), RELEASE toward
         * center = mic closes. Drive on edges so we only toggle the voice engine
         * when the desired state actually changes. */
        if (click && !prev_click) {
            mic_open = true;
            if (!stackchan_voice_is_listening()) stackchan_voice_toggle_listening();
            ESP_LOGI(TAG, "remote stick-DOWN hold -> mic ON (PTT)");
        } else if (!click && prev_click) {
            mic_open = false;
            if (stackchan_voice_is_listening()) stackchan_voice_toggle_listening();
            ESP_LOGI(TAG, "remote stick-DOWN release -> mic OFF (PTT)");
        }

        /* ---- stick UP -> barge-in (abort playback), edge-triggered ---- */
        if (!up_fired && up > MIC_ON_THRESH) {
            up_fired = true;
            stackchan_voice_stream_abort();
            ESP_LOGI(TAG, "remote stick UP -> barge-in (abort playback)");
        } else if (up_fired && up < MIC_OFF_THRESH) {
            up_fired = false;
        }

        /* ---- stick LEFT -> snap a photo (deep + horizontal-dominant) ---- */
        bool left_strong = (left > CAM_ON_THRESH) && (fabsf(ny) < CAM_MAX_VERT);
        if (!cam_fired && left_strong) {
            cam_fired = true;
            bool started = stackchan_camera_capture_async(NULL);
            ESP_LOGI(TAG, "remote stick LEFT -> camera capture (started=%d, left=%.2f ny=%.2f)",
                     (int)started, left, ny);
        } else if (cam_fired && left < CAM_OFF_THRESH) {
            cam_fired = false;
        }
        break;
    }
    }

    prev_click = click;
}

static void espnow_task(void *arg)
{
    (void)arg;

    /* esp_now_init() only needs Wi-Fi *started* (not connected). The node
     * starts Wi-Fi slightly after us, so retry until it succeeds instead of
     * depending on call ordering in app_main. */
    esp_err_t err;
    for (;;) {
        err = esp_now_init();
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "esp_now_init not ready yet (%s); retrying...", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_register_recv_cb failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&primary, &second);

    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, mac);

    ESP_LOGI(TAG, "ESP-NOW remote receiver ready.");
    ESP_LOGI(TAG, "  our STA MAC: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "  listening on Wi-Fi channel %d (must match the remote's channel).", primary);
    ESP_LOGI(TAG, "  app-header size=%d; expecting frames of at least %d bytes.",
             (int)sizeof(espnow_app_hdr_t), (int)sizeof(espnow_app_hdr_t) + ESPNOW_JOY_PAYLOAD_LEN);
    ESP_LOGI(TAG, "  power on the joystick remote near the robot; watch for 'espnow rx' lines.");

    espnow_evt_t evt;
    int64_t s_last_chlog_us = 0;
    for (;;) {
        /* Wake at least every 200ms so we can resume idle on inactivity even
         * when the remote goes silent (no more packets arrive). */
        if (xQueueReceive(s_queue, &evt, pdMS_TO_TICKS(200)) == pdTRUE) {
            espnow_apply_payload(&evt);
        }

        /* DIAG: periodically log our actual radio channel so a mismatch with the
         * remote's fixed TX channel is visible in a fresh capture (boot line
         * scrolls away). ESP-NOW RX only works on this channel. */
        {
            int64_t nowu = esp_timer_get_time();
            if (nowu - s_last_chlog_us > 3000000) {
                s_last_chlog_us = nowu;
                uint8_t ch = 0; wifi_second_chan_t sc = WIFI_SECOND_CHAN_NONE;
                esp_wifi_get_channel(&ch, &sc);
                ESP_LOGW(TAG, "DIAG radio channel=%d (remote TX must match); raw frames so far=(see RAW-RX)", (int)ch);
            }
        }

        /* Ambient look-around resumes once the stick has been quiet (centered
         * or remote powered off) for IDLE_RESUME_MS. The head holds its last
         * pose until then, instead of snapping back to center on release. */
        if (s_engaged && (now_ms() - s_last_active_ms) >= IDLE_RESUME_MS) {
            stackchan_servo_idle_set(true);
            s_engaged = false;
        }
    }
}

esp_err_t stackchan_espnow_remote_start(void)
{
    if (s_queue) {
        return ESP_OK; /* already started */
    }

    s_queue = xQueueCreate(8, sizeof(espnow_evt_t));
    if (!s_queue) {
        ESP_LOGE(TAG, "failed to create espnow queue");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(espnow_task, "espnow_rx", 4096, NULL, 4, NULL, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create espnow task");
        vQueueDelete(s_queue);
        s_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "ESP-NOW remote receiver starting (waiting for Wi-Fi)...");
    return ESP_OK;
}
