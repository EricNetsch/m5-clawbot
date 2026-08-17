/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * StackChan state-driven presentation coordinator -- see header.
 */
#include "stackchan_presentation.h"
#include "stackchan_media.h"
#include "stackchan_ready.h"
#include "stackchan_body.h"   /* stackchan_led_fill / stackchan_thinking_active */
#include "stackchan_servo.h"  /* stackchan_servo_move */
#include "stackchan_voice.h"  /* stackchan_voice_is_streaming / _is_listening */

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

static const char *TAG = "stackchan_pres";

/* SPEAKING head motion: a subtle continuous micro-sway with occasional gentle
 * emphasis gestures -- reads like a person talking rather than a metronome nod.
 * Runs on THIS low-priority presentation task via the UART servo bus (never the
 * shared I2C audio/power bus), so it costs no internal DMA and stays out of the
 * mic-end/speaker-begin window. (Replaces the old fixed-rhythm pitch toggle.) */
#define STK_PRES_SPEAK_SWAY  1
#define POLL_MS              120   /* base poll cadence for state changes       */
#define SPEAK_TICK_MS        90    /* servo update cadence while speaking       */

/* Debug: log emphasis fires + interval (off by default; INFO when 1). */
#define SPK_SWAY_DEBUG       0

/* --- micro-sway + emphasis tuning (all in degrees; keep understated) --- */
#define SPEAK_BASE_YAW    0
#define SPEAK_BASE_PITCH  10       /* neutral speaking pitch (matches s_pose)   */
#define SWAY_YAW_AMP      2.2f     /* peak yaw drift around center              */
#define SWAY_PITCH_AMP    1.8f     /* peak pitch drift around the speaking base */
#define SWAY_MOVE_SPEED   950      /* -> servo ease ~260ms: chases a moving tgt */
/* Slow, NON-integer frequency ratios (Hz) so the summed drift never repeats /
 * looks periodic -- a cheap Perlin-ish wander. */
#define SWAY_YW1          0.13f
#define SWAY_YW2          0.31f
#define SWAY_PW1          0.17f
#define SWAY_PW2          0.23f
/* Emphasis: a slightly larger gesture on jittered beats (no speaker-output
 * amplitude signal is exposed, and plumbing an RMS pipeline into the fragile
 * speaker feed path is out of scope -- so emphasis is timed on an organic
 * jittered interval rather than a clock). It eases in then decays back into the
 * sway, so nothing ever sticks. */
#define EMPH_MIN_MS       1500
#define EMPH_MAX_MS       4000
#define EMPH_FIRST_MIN_MS 800      /* don't gesture the instant speech starts   */
#define EMPH_FIRST_MAX_MS 2000
#define EMPH_DECAY        0.80f    /* per SPEAK_TICK -> ~0.5s settle            */
#define EMPH_STOP         0.6f     /* snap tiny residual offsets to 0 (deg)     */

/* Per-state subtle head pose, degrees relative to center.
 * yaw: -left/+right, pitch: -down/+up. Kept well inside the firmware limits
 * and gentle (the servo layer clamps + eases). */
typedef struct { int yaw; int pitch; int speed; } pose_t;
static const pose_t s_pose[STK_PRES_COUNT] = {
    [STK_PRES_BOOTING]    = {  0,  0, 300 }, /* resting (boot wiggle already ran) */
    [STK_PRES_CONNECTING] = {  0,  6, 300 }, /* slight look up: coming online      */
    [STK_PRES_READY]      = {  0,  8, 300 }, /* relaxed, chin a touch up           */
    [STK_PRES_LISTENING]  = {  0, 16, 400 }, /* attentive lean toward the user     */
    [STK_PRES_THINKING]   = { -8,  8, 300 }, /* pondering tilt                     */
    [STK_PRES_SPEAKING]   = {  0, 10, 400 }, /* engaged (micro-sway wanders around it) */
    [STK_PRES_ERROR]      = {  0,  0, 300 }, /* safe neutral                       */
    [STK_PRES_SHUTDOWN]   = {  0,  0, 300 }, /* (unused: shutdown path owns servo) */
};

static volatile bool s_started = false;
static volatile bool s_shutdown = false;
/* Light-idle: while true the coordinator yields the head/LED (reversibly) so
 * the idle tier can relax the servos + kill the LED without a fight. */
static volatile bool s_idle = false;
static volatile stackchan_ready_state_t s_ready = SYS_BOOTING;
/* Set whenever the coarse ready state changes; consumed when the ring is next
 * safe to update (no voice activity). */
static volatile bool s_led_dirty = true;
static TaskHandle_t s_task = NULL;

static bool activity_state(stk_pres_state_t st)
{
    return st == STK_PRES_LISTENING || st == STK_PRES_THINKING || st == STK_PRES_SPEAKING;
}

/* Compute the effective presentation state from the authoritative sources.
 * Priority: shutdown > speaking > thinking > listening > coarse ready state. */
static stk_pres_state_t effective(void)
{
    if (s_shutdown) {
        return STK_PRES_SHUTDOWN;
    }
    if (stackchan_voice_is_streaming()) {
        return STK_PRES_SPEAKING;
    }
    if (stackchan_thinking_active()) {
        return STK_PRES_THINKING;
    }
    if (stackchan_voice_is_listening()) {
        return STK_PRES_LISTENING;
    }
    switch (s_ready) {
        case SYS_READY:      return STK_PRES_READY;
        case SYS_CONNECTING: return STK_PRES_CONNECTING;
        case SYS_DEGRADED:   return STK_PRES_ERROR;
        case SYS_BOOTING:
        default:             return STK_PRES_BOOTING;
    }
}

stk_pres_state_t stackchan_presentation_lcd_state(void)
{
    return effective();
}

static void apply_servo_pose(stk_pres_state_t st)
{
    if ((int)st < 0 || st >= STK_PRES_COUNT) {
        return;
    }
    const pose_t *p = &s_pose[st];
    stackchan_servo_move(p->yaw, p->pitch, p->speed);
}

/* Coarse-state ring color. Voice sub-states are NOT handled here (owned by the
 * timing-tuned voice LED paths); we only reach this for coarse states. */
static void apply_coarse_led(stk_pres_state_t st)
{
    switch (st) {
        case STK_PRES_BOOTING:    stackchan_led_fill(40, 16, 0); break; /* amber: waking     */
        case STK_PRES_CONNECTING: stackchan_led_fill(40, 16, 0); break; /* amber: connecting */
        case STK_PRES_READY:      stackchan_led_fill(0, 0, 0);   break; /* off: idle         */
        case STK_PRES_ERROR:      stackchan_led_fill(40, 0, 0);  break; /* red: degraded     */
        default: break; /* activity/shutdown: leave the ring to its owner */
    }
}

static void on_ready_change(stackchan_ready_state_t state, void *ctx)
{
    (void)ctx;
    s_ready = state;
    s_led_dirty = true; /* refresh the ring when it is next safe (no activity) */
    if (s_task != NULL) {
        xTaskNotifyGive(s_task); /* wake for a prompt physical-cue update */
    }
}

static void pres_task(void *arg)
{
    (void)arg;
    stk_pres_state_t last = STK_PRES_COUNT; /* invalid -> forces first apply */
    int64_t sway_start_us = 0;   /* esp_timer time SPEAKING began              */
    int64_t next_emph_us = 0;    /* when the next emphasis gesture may fire     */
    float   em_yaw = 0.0f;       /* live emphasis offset (decays each tick)     */
    float   em_pitch = 0.0f;

    for (;;) {
        /* Light idle: yield the head/LED entirely. Keep forcing `last` invalid
         * so that on wake (s_idle cleared) the next iteration re-applies the
         * current state's pose, and leave s_led_dirty set so the ring refreshes
         * too. Do NOT drive any hardware here -- the idle tier owns it. */
        if (s_idle) {
            last = STK_PRES_COUNT;
            s_led_dirty = true;
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(POLL_MS));
            continue;
        }

        stk_pres_state_t st = effective();
        bool activity = activity_state(st);

        /* Head pose on any state change (UART -- safe w.r.t. the audio bus).
         * Skip while shutting down so we never re-torque a relaxed head. */
        if (st != last) {
            if (st != STK_PRES_SHUTDOWN) {
                apply_servo_pose(st);
                ESP_LOGI(TAG, "pres -> %s (pose %d,%d)", stk_pres_state_name(st),
                         s_pose[st].yaw, s_pose[st].pitch);
            } else {
                ESP_LOGI(TAG, "pres -> %s (yield hardware to shutdown path)",
                         stk_pres_state_name(st));
            }
            last = st;
            if (st == STK_PRES_SPEAKING) {
                /* Reset the sway clock + emphasis state on entry; schedule the
                 * first gesture a beat in so it doesn't fire the instant speech
                 * starts. The base (0,10) pose was just applied above; the sway
                 * wanders around it, and on exit the next state's pose apply
                 * (e.g. READY 0,8 at slow speed) glides it home cleanly. */
                sway_start_us = esp_timer_get_time();
                em_yaw = 0.0f;
                em_pitch = 0.0f;
                next_emph_us = sway_start_us +
                    (int64_t)(EMPH_FIRST_MIN_MS +
                              (esp_random() % (EMPH_FIRST_MAX_MS - EMPH_FIRST_MIN_MS))) * 1000;
            }
        }

        /* Coarse connectivity ring: only when a coarse state is showing AND a
         * ready change is pending. Deferred while any voice activity is in
         * flight so no LED I2C burst lands in the mic-end/speaker-begin window. */
        if (s_led_dirty && !activity && st != STK_PRES_SHUTDOWN) {
            apply_coarse_led(st);
            s_led_dirty = false;
        }

#if STK_PRES_SPEAK_SWAY
        if (st == STK_PRES_SPEAKING) {
            int64_t now_us = esp_timer_get_time();
            float t = (float)(now_us - sway_start_us) / 1000000.0f;
            const float TWO_PI = 2.0f * (float)M_PI;

            /* Micro-sway: two slow sines per axis with non-integer frequency
             * ratios -> a smooth, slowly-wandering drift that never looks
             * periodic (a person holding gaze with tiny drifts, not a nod). */
            float yaw_sway = SWAY_YAW_AMP *
                (0.62f * sinf(TWO_PI * SWAY_YW1 * t) +
                 0.38f * sinf(TWO_PI * SWAY_YW2 * t + 1.7f));
            float pitch_sway = SWAY_PITCH_AMP *
                (0.60f * sinf(TWO_PI * SWAY_PW1 * t + 0.9f) +
                 0.40f * sinf(TWO_PI * SWAY_PW2 * t));

            /* Emphasis gesture on a jittered interval: mostly a gentle downward
             * dip (nod-like accent), sometimes a small chin-up accent, plus a
             * little yaw so it reads as a gesture, then it decays back in. */
            if (now_us >= next_emph_us) {
                uint32_t r = esp_random();
                if ((r & 0x3u) == 0u) {
                    em_pitch = 2.0f + (float)((r >> 2) % 20u) / 10.0f;    /* +2.0..+4.0 up  */
                } else {
                    em_pitch = -(2.5f + (float)((r >> 2) % 30u) / 10.0f); /* -5.5..-2.5 dip */
                }
                em_yaw = (((r >> 8) & 1u) ? 1.0f : -1.0f) *
                         (1.5f + (float)((r >> 9) % 25u) / 10.0f);        /* +/-1.5..4.0    */
                uint32_t iv = EMPH_MIN_MS + (esp_random() % (EMPH_MAX_MS - EMPH_MIN_MS));
                next_emph_us = now_us + (int64_t)iv * 1000;
#if SPK_SWAY_DEBUG
                ESP_LOGI(TAG, "emphasis y=%.1f p=%.1f next=%ums",
                         (double)em_yaw, (double)em_pitch, (unsigned)iv);
#endif
            }

            int yaw_cmd   = (int)lroundf((float)SPEAK_BASE_YAW   + yaw_sway   + em_yaw);
            int pitch_cmd = (int)lroundf((float)SPEAK_BASE_PITCH + pitch_sway + em_pitch);
            stackchan_servo_move(yaw_cmd, pitch_cmd, SWAY_MOVE_SPEED);
#if SPK_SWAY_DEBUG
            ESP_LOGI(TAG, "sway t=%.2f -> y=%d p=%d", (double)t, yaw_cmd, pitch_cmd);
#endif

            /* Decay the emphasis offset back into the sway (no stuck offset). */
            em_yaw   *= EMPH_DECAY;
            em_pitch *= EMPH_DECAY;
            if (fabsf(em_yaw)   < EMPH_STOP) em_yaw = 0.0f;
            if (fabsf(em_pitch) < EMPH_STOP) em_pitch = 0.0f;

            /* Faster tick while speaking for continuous motion; still blocks
             * (yields), waking early on a state-change notify so exit is prompt. */
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SPEAK_TICK_MS));
            continue;
        }
#endif

        /* Block up to POLL_MS, waking early on a ready-change notify. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(POLL_MS));
    }
}

void stackchan_presentation_note_shutdown(void)
{
    s_shutdown = true;
    if (s_task != NULL) {
        xTaskNotifyGive(s_task);
    }
}

void stackchan_presentation_note_idle(bool sleeping)
{
    s_idle = sleeping;
    s_led_dirty = true; /* refresh the ring on resume */
    if (s_task != NULL) {
        xTaskNotifyGive(s_task);
    }
}

void stackchan_presentation_start(void)
{
    if (s_started) {
        return;
    }
    s_started = true;
    s_ready = stackchan_ready_get();
    s_led_dirty = true;

    /* Explicitly install the flash/SPIFFS provider (swap here later for SD). */
    stk_media_set_provider(stk_media_flash_provider());

    esp_err_t err = stackchan_ready_on_change(on_ready_change, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ready subscribe failed: %d (falling back to poll only)", (int)err);
    }

    /* Stack in PSRAM (like the camera worker): this task is created at boot,
     * before Wi-Fi/TLS fragment RAM, and does no DMA/flash-cache-disabling work
     * on its stack -- so keeping it out of internal SRAM leaves the scarce
     * contiguous internal-DMA block (that the two WS legs + speaker need)
     * exactly as it was on c8d4dcb. The TCB stays internal (WithCaps forces
     * that). Core 0 -- core 1 hosts the avatar render loop. */
    if (xTaskCreatePinnedToCoreWithCaps(pres_task, "pres", 3072, NULL, 2, &s_task, 0,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGE(TAG, "failed to create presentation task");
        s_task = NULL;
    }
    ESP_LOGI(TAG, "presentation coordinator started (provider=%s)",
             stk_media_get_provider()->name);
}
