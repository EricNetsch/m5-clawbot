/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * StackChan state-driven presentation coordinator.
 *
 * SUBSCRIBES to the real device state (stackchan_ready SYS_* states) and the
 * voice sub-states (listening / thinking / speaking) -- it never duplicates or
 * owns that state, it only maps it to a presentation:
 *
 *   effective presentation state = f(ready coarse state, voice sub-states)
 *     shutdown > speaking > thinking > listening > {ready coarse state}
 *
 * and drives the three presentation surfaces per state:
 *   - LCD  : the avatar render loop (in stackchan_boot.cpp) reads
 *            stackchan_presentation_lcd_state() and pulls the matching frame
 *            sequence from the swappable media provider (stackchan_media.*).
 *   - SERVO: a subtle per-state head pose on the Feetech bus (UART1 -- fully
 *            separate from the fragile shared I2C audio/power bus).
 *   - LED  : the 12-LED ring shows the COARSE connectivity states (booting /
 *            connecting / ready / error). The voice sub-state LED cues
 *            (blue=listening, pink=thinking, green=speaking) remain owned by
 *            the existing, timing-tuned voice paths; the coordinator defers to
 *            them and only writes the ring on coarse-state changes while no
 *            voice activity is in flight -- this deliberately keeps every LED
 *            I2C burst out of the delicate mic-end / speaker-begin window that
 *            history showed can wedge the shared bus.
 *
 * The coordinator holds NO scarce internal DMA (LCD frames decode from a PSRAM
 * buffer; servo is UART; LED is a gated single burst) and runs on a low
 * priority task that yields, so it never starves the voice / camera / WS legs.
 */
#pragma once

#include <stdbool.h>

#include "stackchan_media.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start the coordinator: cache the current ready state, subscribe to ready
 * transitions, and spawn the low-priority presentation task. Call once, after
 * stackchan_ready_init() + stackchan_boot() + stackchan_servo_start() +
 * stackchan_voice_start(). Idempotent. */
void stackchan_presentation_start(void);

/* The effective presentation state for the LCD RIGHT NOW, computed live from
 * the ready state + voice sub-states. Cheap + lock-free; safe to call from the
 * avatar render task every frame. Safe (returns a sane default) before start. */
stk_pres_state_t stackchan_presentation_lcd_state(void);

/* Tell the coordinator a graceful shutdown has begun so it stops driving the
 * head/LEDs and lets the shutdown path (goodnight screen + servo relax) own the
 * hardware unopposed. Call at the very start of the shutdown sequence. */
void stackchan_presentation_note_shutdown(void);

/* Enter (true) / exit (false) light-idle. While idle the coordinator stops
 * driving the head pose, sway, and LED ring so the idle tier can relax the
 * servos + turn the LED off without the coordinator fighting it. On exit it
 * re-applies the current state's pose + LED. Unlike note_shutdown this is
 * REVERSIBLE. */
void stackchan_presentation_note_idle(bool sleeping);

#ifdef __cplusplus
}
#endif
