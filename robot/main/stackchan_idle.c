/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * StackChan tiered idle/sleep coordinator -- see stackchan_idle.h.
 */

#include "stackchan_idle.h"

#include "stackchan_body.h"                     /* avatar_set / display_brightness / display_sleep / led_fill */
#include "stackchan_servo.h"                    /* servo relax / wake */
#include "stackchan_presentation.h"             /* note_idle: quiesce/resume the coordinator */
#include "stackchan_voice.h"                    /* is_listening / is_streaming */
#include "esp_openclaw_node_stackchan_rgb_cmd.h"/* graceful power-down (Tier 2) */

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "esp_console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "stackchan_idle";

/* ---- Timeouts (compile-time overridable for a fast test path) ------------- *
 * Build with e.g. -DIDLE_T1_MS=8000 -DIDLE_T2_MS=20000 to exercise all the
 * transitions in seconds instead of minutes. The committed defaults are the
 * REAL 5 min / 30 min. The `idle` console command also forces transitions
 * without any wait. */
#ifndef IDLE_T1_MS
#define IDLE_T1_MS (5u * 60u * 1000u)   /* light idle after 5 minutes */
#endif
#ifndef IDLE_T2_MS
#define IDLE_T2_MS (30u * 60u * 1000u)  /* full power-down after 30 minutes */
#endif

/* Timer-task cadence. Short enough that the countdown is responsive and a
 * missed activity edge is still caught quickly; long enough to be near-zero
 * CPU. Wake is driven by an explicit notify (instant), not this poll. */
#define IDLE_TICK_MS 1000

/* LCD backlight level to restore on wake (matches the boot brightness). */
#define IDLE_ACTIVE_BRIGHTNESS 51

/* WiFi power-save: ACTIVE keeps the radio fully awake (best WS reliability +
 * lowest wake latency for an incoming turn); TIER1 enables modem power-save so
 * the radio sleeps between beacons WITHOUT dropping the association / the two
 * WS legs. We deliberately use MIN_MODEM (wakes every beacon) rather than
 * MAX_MODEM (wakes only on DTIM, higher latency): per Eric's spec, connection
 * reliability + instant wake win over the last few mA, and MIN_MODEM is the
 * conservative mode proven to hold long-lived TCP/WS sockets up. Bump the
 * TIER1 value to WIFI_PS_MAX_MODEM here if a future test confirms the legs
 * survive it. */
#define IDLE_WIFI_PS_ACTIVE  WIFI_PS_NONE
#define IDLE_WIFI_PS_TIER1   WIFI_PS_MIN_MODEM

static volatile stk_idle_mode_t s_mode = STK_IDLE_ACTIVE;
static volatile bool s_started = false;
static volatile bool s_activity = false;    /* set by note_activity, cleared by task */
static volatile bool s_force_t1 = false;    /* console: idle now  */
static volatile bool s_force_wake = false;  /* console: idle wake */
static volatile bool s_force_t2 = false;    /* console: idle tier2 */
static TaskHandle_t s_task = NULL;

static void wifi_set_ps_safe(wifi_ps_type_t ps)
{
    esp_err_t err = esp_wifi_set_ps(ps);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_ps(%d) -> %s (non-fatal)", (int)ps, esp_err_to_name(err));
    }
}

/* True if any activity source is CURRENTLY in flight (polled backstop; the
 * instant path is note_activity). Voice covers LISTENING/THINKING/SPEAKING and
 * incoming gateway audio playback (streaming). */
static bool activity_in_flight(void)
{
    return stackchan_voice_is_listening() ||
           stackchan_voice_is_streaming() ||
           stackchan_thinking_active();
}

/* ------------------------------------------------------------------ *
 * TIER 1 enter: shed screen / render / servo / LED / camera + WiFi PS. *
 * Order matters: quiesce the presentation coordinator FIRST so it stops *
 * driving the head/LED, THEN relax the servos (no re-torque race).      *
 * ------------------------------------------------------------------ */
static void enter_tier1(void)
{
    if (s_mode == STK_IDLE_TIER1) {
        return;
    }
    ESP_LOGI(TAG, "-> TIER1 (light idle): backlight+panel off, render loop stopped, "
                  "servos relaxed, camera dormant, LED off, WiFi modem power-save on; "
                  "connection kept HOT for instant wake");

    /* 1. Coordinator yields the head/LED (like the shutdown yield, but
     *    reversible) so it will not fight us or re-torque the servos. */
    stackchan_presentation_note_idle(true);

    /* 2. Stop the avatar/animation render loop (the ~90ms/frame redraw). */
    stackchan_avatar_set(false);
    vTaskDelay(pdMS_TO_TICKS(130)); /* let any in-flight frame finish */

    /* 3. LCD backlight OFF, then panel to sleep (SPI -- no I2C traffic). */
    stackchan_display_brightness(0);
    stackchan_display_sleep(true);

    /* 4. RGB LED off (single quiet I2C burst; bus is idle here). */
    stackchan_led_fill(0, 0, 0);

    /* 5. Relax the servos (release holding torque -- UART bus). */
    stackchan_servo_relax();

    /* 6. Camera: nothing to do. It is deinited right after every shot, so at
     *    idle it already holds ZERO internal DMA and its worker is blocked on a
     *    semaphore. Next capture lazily re-inits (activity would wake us first).
     *    We do NOT cut the AXP2101 camera LDO here: poking the PMIC over the
     *    fragile shared I2C bus at idle risks the very connection we must keep
     *    alive, and the sensor's residual draw is negligible next to the wins
     *    above. */

    /* 7. WiFi modem power-save (keeps the association + both WS legs up). */
    wifi_set_ps_safe(IDLE_WIFI_PS_TIER1);

    s_mode = STK_IDLE_TIER1;
}

/* ------------------------------------------------------------------ *
 * WAKE (Tier 1 -> ACTIVE): restore everything, fast.                   *
 * ------------------------------------------------------------------ */
static void do_wake(const char *why)
{
    if (s_mode != STK_IDLE_TIER1) {
        return;
    }
    int64_t t0 = esp_timer_get_time();

    /* 1. Radio fully awake again first, so an incoming turn streams smoothly. */
    wifi_set_ps_safe(IDLE_WIFI_PS_ACTIVE);

    /* 2. Panel + backlight back on. */
    stackchan_display_sleep(false);
    stackchan_display_brightness(IDLE_ACTIVE_BRIGHTNESS);

    /* 3. Re-torque the servos + recenter so the coordinator can pose the head. */
    stackchan_servo_wake();

    /* 4. Coordinator resumes: it re-applies the current state's pose + LED. */
    stackchan_presentation_note_idle(false);

    /* 5. Resume the avatar/animation render loop. (Camera stays lazy.) */
    stackchan_avatar_set(true);

    s_mode = STK_IDLE_ACTIVE;
    ESP_LOGI(TAG, "WAKE -> ACTIVE (%s) in %lld ms",
             why ? why : "activity", (long long)((esp_timer_get_time() - t0) / 1000));
}

/* ------------------------------------------------------------------ *
 * TIER 2: reuse the EXISTING graceful power-down. Does not return.     *
 * ------------------------------------------------------------------ */
static void enter_tier2(void)
{
    s_mode = STK_IDLE_TIER2;
    ESP_LOGW(TAG, "-> TIER2 (deep sleep): running existing graceful power-down");
    /* Reuse the exact path the power-button short-press / `power off` use:
     * note_shutdown -> goodnight -> recenter + relax servos -> AXP soft off. */
    stackchan_shutdown_gracefully();
    /* Normally never returns (rails drop). If it somehow does, stay off-air. */
}

static void idle_task(void *arg)
{
    (void)arg;
    int64_t last_activity_us = esp_timer_get_time();

    for (;;) {
        int64_t now = esp_timer_get_time();

        /* Console-forced transitions (fast test path). */
        if (s_force_wake) {
            s_force_wake = false;
            do_wake("console");
            last_activity_us = now;
        }
        if (s_force_t2) {
            s_force_t2 = false;
            enter_tier2(); /* no return */
        }
        if (s_force_t1) {
            s_force_t1 = false;
            if (s_mode == STK_IDLE_ACTIVE) {
                enter_tier1();
            }
        }

        /* Activity edge (instant wake path) + polled backstop. */
        bool act = false;
        if (s_activity) { s_activity = false; act = true; }
        if (activity_in_flight()) { act = true; }
        if (act) {
            last_activity_us = now;
            if (s_mode == STK_IDLE_TIER1) {
                do_wake("activity");
            }
        }

        int64_t idle_ms = (now - last_activity_us) / 1000;

        if (s_mode == STK_IDLE_ACTIVE) {
            if (idle_ms >= (int64_t)IDLE_T1_MS) {
                enter_tier1();
            }
        } else if (s_mode == STK_IDLE_TIER1) {
            if (idle_ms >= (int64_t)IDLE_T2_MS) {
                enter_tier2(); /* no return */
            }
        }

        /* Block, but wake instantly on a notify (activity / console). */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IDLE_TICK_MS));
    }
}

void stackchan_idle_note_activity(void)
{
    if (!s_started) {
        return;
    }
    s_activity = true;
    if (s_task != NULL) {
        xTaskNotifyGive(s_task);
    }
}

stk_idle_mode_t stackchan_idle_mode(void)
{
    return s_mode;
}

void stackchan_idle_start(void)
{
    if (s_started) {
        return;
    }

    /* Establish the ACTIVE WiFi power-save baseline (fully awake for best WS
     * reliability + wake latency). Non-fatal if WiFi isn't started yet. */
    wifi_set_ps_safe(IDLE_WIFI_PS_ACTIVE);

    s_mode = STK_IDLE_ACTIVE;
    s_activity = false;

    /* Low-priority timer task; stack in PSRAM (same pattern as the camera /
     * presentation tasks) so it costs ZERO scarce internal SRAM. It does no
     * flash-cache-disabling work on its stack. Core 0 (core 1 hosts the avatar
     * loop + speaker mixer). */
    if (xTaskCreatePinnedToCoreWithCaps(idle_task, "stk_idle", 4096, NULL, 2, &s_task, 0,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGE(TAG, "failed to create idle task");
        s_task = NULL;
        return;
    }
    s_started = true;
    ESP_LOGI(TAG, "idle coordinator started (T1=%ums light-idle, T2=%ums power-down)",
             (unsigned)IDLE_T1_MS, (unsigned)IDLE_T2_MS);
}

/* ------------------------------ console -------------------------------- */
static int handle_idle_cmd(int argc, char **argv)
{
    const char *sub = (argc >= 2) ? argv[1] : "status";
    if (strcmp(sub, "now") == 0) {
        printf("idle: forcing TIER1 (light idle)\n");
        s_force_t1 = true;
        if (s_task) xTaskNotifyGive(s_task);
        return 0;
    }
    if (strcmp(sub, "wake") == 0) {
        printf("idle: forcing WAKE -> ACTIVE\n");
        s_force_wake = true;
        if (s_task) xTaskNotifyGive(s_task);
        return 0;
    }
    if (strcmp(sub, "tier2") == 0) {
        printf("idle: forcing TIER2 (graceful power-down; device will power off)\n");
        s_force_t2 = true;
        if (s_task) xTaskNotifyGive(s_task);
        return 0;
    }
    if (strcmp(sub, "status") == 0) {
        const char *m = (s_mode == STK_IDLE_ACTIVE) ? "ACTIVE"
                      : (s_mode == STK_IDLE_TIER1)  ? "TIER1"
                                                    : "TIER2";
        printf("idle: mode=%s started=%d activity_in_flight=%d T1=%ums T2=%ums\n",
               m, (int)s_started, (int)activity_in_flight(),
               (unsigned)IDLE_T1_MS, (unsigned)IDLE_T2_MS);
        return 0;
    }
    printf("usage: idle now | wake | tier2 | status\n");
    return 1;
}

int stackchan_idle_register_command(void)
{
    esp_console_cmd_t cmd = {0};
    cmd.command = "idle";
    cmd.help = "Tiered idle/sleep: force transitions for fast testing.";
    cmd.hint = "now|wake|tier2|status";
    cmd.func = handle_idle_cmd;
    return (int)esp_console_cmd_register(&cmd);
}
