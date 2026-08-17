/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * StackChan tiered IDLE / SLEEP mode (queue item #5).
 *
 * Two inactivity tiers driven by a single low-priority timer task:
 *
 *   TIER 1 -- "light idle" after IDLE_T1_MS (default 5 min) of no activity.
 *     The connection stays HOT (both WS legs keep pinging) so an incoming
 *     gateway turn wakes the device instantly. To conserve battery + reduce
 *     component wear we shed everything that is safe to shed:
 *       - LCD backlight OFF + panel sleep  (biggest win; saves backlight wear)
 *       - avatar/animation render loop STOPPED (no ~90ms/frame redraw -> CPU)
 *       - servos relaxed (torque released -> no holding current)
 *       - camera dormant (already deinited between shots -> zero DMA held)
 *       - RGB LED off
 *       - WiFi modem power-save enabled (radio saves power WITHOUT dropping the
 *         WS legs; MIN_MODEM -- see the .c for why not MAX_MODEM)
 *     The speaker's permanently-claimed I2S DMA (c8d4dcb) is DELIBERATELY kept
 *     so the first post-wake reply plays instantly and cleanly.
 *
 *   TIER 2 -- "deep sleep = full off" after IDLE_T2_MS (default 30 min).
 *     Runs the EXISTING graceful power-down (goodnight -> recenter+relax servos
 *     -> AXP2101 soft power-off). Eric power-cycles / charges to bring it back.
 *
 * ANY activity (voice LISTENING/THINKING/SPEAKING, camera capture, PTT/button,
 * an incoming gateway turn/audio playback) resets BOTH timers and, if we are in
 * Tier 1, wakes back to full ACTIVE fast (well under a second).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STK_IDLE_ACTIVE = 0,  /* fully awake */
    STK_IDLE_TIER1,       /* light idle (screen/render/servo/LED shed, connected) */
    STK_IDLE_TIER2,       /* transient: graceful power-down in progress */
} stk_idle_mode_t;

/* Start the idle coordinator: spawn the low-priority timer task and set the
 * active WiFi power-save baseline. Call once, late in app_main (after wifi
 * start + boot + servo + voice + presentation are up). Idempotent. */
void stackchan_idle_start(void);

/* Signal that an activity occurred. Resets both inactivity timers and, if the
 * device is in Tier 1, requests an immediate wake back to ACTIVE. Safe to call
 * from any task/context, and before stackchan_idle_start() (no-op then). Kept
 * tiny + non-blocking (sets a flag + notifies the task). */
void stackchan_idle_note_activity(void);

/* Current mode (lock-free snapshot). */
stk_idle_mode_t stackchan_idle_mode(void);

/* Register the `idle` serial console command (now/wake/tier2/status) for a
 * fast verification path that does not require waiting the real 5/30 minutes. */
int stackchan_idle_register_command(void);

#ifdef __cplusplus
}
#endif
