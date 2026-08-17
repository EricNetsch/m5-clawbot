/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Flash/SPIFFS media provider -- see stackchan_media.h.
 *
 * Maps each presentation state to a sub-range of the staged avatar frames
 * (/spiffs/f001..f070.jpg) plus a playback rate. The frames are an abstract
 * glowing "orb": the early/late frames are a calm nebula (no star = "asleep"),
 * and a bright star materialises through the middle of the loop (= "awake /
 * present"). That content drives the per-state mapping below:
 *
 *   BOOTING     nebula only, slow, dim  -> "waking up"
 *   CONNECTING  nebula building         -> "coming online"
 *   READY       full breathe loop       -> calm idle face (unchanged behaviour)
 *   LISTENING   star held, steady       -> attentive
 *   SPEAKING    star region, fast pulse -> "talking" (distinct animation)
 *   ERROR       nebula fade, slow, dim  -> degraded (red LED carries it)
 *   THINKING    dedicated t001..t050.jpg clip -> "pondering" (distinct animation)
 *   SHUTDOWN    (LCD owned by goodnight; falls back to the idle loop)
 *
 * Only the RANGES live here; swapping to an SD/MJPEG backend later means
 * implementing the same three callbacks against a different source and calling
 * stk_media_set_provider() -- no change to the mapping or the render loop.
 */
#include "stackchan_media.h"

#include <stdio.h>
#include <string.h>

/* First/last 1-based frame number in /spiffs and the playback rate per state. */
typedef struct {
    int start; /* 1-based inclusive */
    int end;   /* 1-based inclusive */
    int fps;
} flash_seq_t;

/* f001..f070 are present on the "storage" SPIFFS partition. */
#define FLASH_FRAME_MIN 1
#define FLASH_FRAME_MAX 70

/* SPEAKING has its OWN distinct frame set: s001..s050.jpg (separate video).
 * It does NOT share the f-frame idle pool. */
#define SPEAK_FRAME_MIN 1
#define SPEAK_FRAME_MAX 50
#define SPEAK_FPS       15

/* THINKING has its OWN distinct frame set: t001..t050.jpg (separate video).
 * It does NOT share the f-frame idle pool. */
#define THINK_FRAME_MIN 1
#define THINK_FRAME_MAX 50
#define THINK_FPS       15

static const flash_seq_t s_seq[STK_PRES_COUNT] = {
    [STK_PRES_BOOTING]    = { .start = 1,  .end = 18, .fps = 8  },
    [STK_PRES_CONNECTING] = { .start = 1,  .end = 30, .fps = 10 },
    [STK_PRES_READY]      = { .start = 1,  .end = 70, .fps = 12 },
    [STK_PRES_LISTENING]  = { .start = 30, .end = 55, .fps = 10 },
    /* THINKING uses the dedicated t001..t050.jpg set (see flash_get_frame);
     * start/end here mirror that count so frame_count/fps stay correct. */
    [STK_PRES_THINKING]   = { .start = 1,  .end = THINK_FRAME_MAX, .fps = THINK_FPS },
    /* SPEAKING uses the dedicated s001..s050.jpg set (see flash_get_frame);
     * start/end here mirror that count so frame_count/fps stay correct. */
    [STK_PRES_SPEAKING]   = { .start = 1,  .end = SPEAK_FRAME_MAX, .fps = SPEAK_FPS },
    [STK_PRES_ERROR]      = { .start = 60, .end = 70, .fps = 6  },
    [STK_PRES_SHUTDOWN]   = { .start = 1,  .end = 70, .fps = 12 },
};

static const flash_seq_t *seq_for(stk_pres_state_t st)
{
    if ((int)st < 0 || st >= STK_PRES_COUNT) {
        return &s_seq[STK_PRES_READY];
    }
    const flash_seq_t *s = &s_seq[st];
    if (s->end < s->start || s->fps <= 0) {
        return &s_seq[STK_PRES_READY]; /* table hole -> calm default */
    }
    return s;
}

static int flash_frame_count(const stk_media_provider_t *self, stk_pres_state_t st)
{
    (void)self;
    const flash_seq_t *s = seq_for(st);
    return s->end - s->start + 1;
}

static int flash_fps(const stk_media_provider_t *self, stk_pres_state_t st)
{
    (void)self;
    return seq_for(st)->fps;
}

static int flash_get_frame(const stk_media_provider_t *self, stk_pres_state_t st,
                           int idx, uint8_t *buf, size_t cap)
{
    (void)self;
    if (buf == NULL || cap == 0) {
        return -1;
    }
    const flash_seq_t *s = seq_for(st);
    int count = s->end - s->start + 1;
    if (count <= 0) {
        return -1;
    }
    if (idx < 0) {
        idx = 0;
    }

    char path[48];
    if (st == STK_PRES_SPEAKING) {
        /* Dedicated speaking clip: s001..s050.jpg, 1-based, wrapped. */
        int frame = SPEAK_FRAME_MIN + (idx % count);
        if (frame < SPEAK_FRAME_MIN) frame = SPEAK_FRAME_MIN;
        if (frame > SPEAK_FRAME_MAX) frame = SPEAK_FRAME_MAX;
        snprintf(path, sizeof(path), "/spiffs/s%03d.jpg", frame);
    } else if (st == STK_PRES_THINKING) {
        /* Dedicated thinking clip: t001..t050.jpg, 1-based, wrapped. */
        int frame = THINK_FRAME_MIN + (idx % count);
        if (frame < THINK_FRAME_MIN) frame = THINK_FRAME_MIN;
        if (frame > THINK_FRAME_MAX) frame = THINK_FRAME_MAX;
        snprintf(path, sizeof(path), "/spiffs/t%03d.jpg", frame);
    } else {
        int frame = s->start + (idx % count); /* wrap within the state's range */
        if (frame < FLASH_FRAME_MIN) frame = FLASH_FRAME_MIN;
        if (frame > FLASH_FRAME_MAX) frame = FLASH_FRAME_MAX;
        snprintf(path, sizeof(path), "/spiffs/f%03d.jpg", frame);
    }
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        return -1;
    }
    size_t n = fread(buf, 1, cap, f);
    fclose(f);
    if (n == 0 || n >= cap) {
        return -1; /* empty or didn't fit the buffer */
    }
    return (int)n;
}

static const stk_media_provider_t s_flash_provider = {
    .name = "flash-spiffs",
    .frame_count = flash_frame_count,
    .fps = flash_fps,
    .get_frame = flash_get_frame,
    .ctx = NULL,
};

const stk_media_provider_t *stk_media_flash_provider(void)
{
    return &s_flash_provider;
}

/* Active provider. Lazily defaults to flash so the avatar loop has a source
 * from the very first frame (the coordinator installs/keeps this later). */
static const stk_media_provider_t *s_active = NULL;

void stk_media_set_provider(const stk_media_provider_t *p)
{
    s_active = (p != NULL) ? p : &s_flash_provider;
}

const stk_media_provider_t *stk_media_get_provider(void)
{
    const stk_media_provider_t *p = s_active;
    if (p == NULL) {
        p = &s_flash_provider;
    }
    return p;
}

const char *stk_pres_state_name(stk_pres_state_t st)
{
    switch (st) {
        case STK_PRES_BOOTING:    return "BOOTING";
        case STK_PRES_CONNECTING: return "CONNECTING";
        case STK_PRES_READY:      return "READY";
        case STK_PRES_LISTENING:  return "LISTENING";
        case STK_PRES_THINKING:   return "THINKING";
        case STK_PRES_SPEAKING:   return "SPEAKING";
        case STK_PRES_ERROR:      return "ERROR";
        case STK_PRES_SHUTDOWN:   return "SHUTDOWN";
        default:                  return "?";
    }
}
