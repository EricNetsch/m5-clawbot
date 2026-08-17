/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * StackChan media-provider abstraction (state-driven presentation layer).
 *
 * The device shows a different visual per presentation state (see
 * stk_pres_state_t). WHERE those frames come from is decoupled behind a small
 * provider vtable so the frame backend is swappable WITHOUT touching the
 * state->presentation mapping or the LCD render loop:
 *
 *   NOW:   flash/SPIFFS provider -- the staged f001..f070.jpg frames baked into
 *          the "storage" SPIFFS partition (stk_media_flash_provider()).
 *   LATER: an SD-card MJPEG provider can implement the SAME three callbacks
 *          (frame_count / fps / get_frame) over a demuxed .mjpg per state and
 *          be installed with stk_media_set_provider() -- the avatar render loop
 *          and the presentation coordinator need ZERO changes.
 *
 * A provider only supplies JPEG bytes for (state, frame index) on demand; it
 * NEVER owns the LCD or any hardware. Frames are decoded straight to the SPI
 * display from a PSRAM read buffer, so this layer consumes NO scarce internal
 * DMA (the hard-won budget behind the speaker/camera stays untouched).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Coarse, human-meaningful presentation states. Derived by the coordinator
 * (stackchan_presentation.*) from the real stackchan_ready state machine plus
 * the voice sub-states (listening / thinking / speaking). These are the ONLY
 * states the media provider and the render loop know about. */
typedef enum {
    STK_PRES_BOOTING = 0, /* powering up; nothing connected yet          */
    STK_PRES_CONNECTING,  /* wifi/tailscale/a leg up, not fully ready    */
    STK_PRES_READY,       /* fully ready + idle: the calm avatar face     */
    STK_PRES_LISTENING,   /* mic/PTT active: attentive look               */
    STK_PRES_THINKING,    /* waiting for the reply (LCD owned by orb)      */
    STK_PRES_SPEAKING,    /* reply audio playing: animated "talking" look  */
    STK_PRES_ERROR,       /* was ready, a required leg dropped (DEGRADED)  */
    STK_PRES_SHUTDOWN,    /* graceful power-off (LCD owned by goodnight)    */
    STK_PRES_COUNT,
} stk_pres_state_t;

/* Human-readable state name (for logs). */
const char *stk_pres_state_name(stk_pres_state_t st);

/* A media provider: supplies JPEG frames for a presentation state. All three
 * callbacks are pure/read-only w.r.t. hardware and may be called from the LCD
 * render task. Implement this to add a new backend (e.g. SD/MJPEG). */
typedef struct stk_media_provider stk_media_provider_t;
struct stk_media_provider {
    const char *name;

    /* Number of frames in this state's sequence (>= 1). A state with no
     * dedicated media should return the idle sequence's count so the render
     * loop always has something calm to show. */
    int (*frame_count)(const stk_media_provider_t *self, stk_pres_state_t st);

    /* Playback rate (frames per second, >= 1) for this state's sequence. */
    int (*fps)(const stk_media_provider_t *self, stk_pres_state_t st);

    /* Load frame @p idx (0-based, wrapped modulo frame_count) of @p st into the
     * caller-provided buffer (which lives in PSRAM). Returns the JPEG byte
     * length on success, or <= 0 on error (render loop then skips the frame). */
    int (*get_frame)(const stk_media_provider_t *self, stk_pres_state_t st,
                     int idx, uint8_t *buf, size_t cap);

    void *ctx;
};

/* Install the active provider (swap the backend). NULL restores the built-in
 * flash/SPIFFS provider. Thread-safe (single pointer swap). */
void stk_media_set_provider(const stk_media_provider_t *p);

/* The active provider. Never NULL: lazily defaults to the flash provider so the
 * avatar loop works from the first frame, before the coordinator starts. */
const stk_media_provider_t *stk_media_get_provider(void);

/* The built-in flash/SPIFFS provider over /spiffs/f001..f070.jpg. */
const stk_media_provider_t *stk_media_flash_provider(void);

#ifdef __cplusplus
}
#endif
