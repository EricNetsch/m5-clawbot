/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * StackChan voice loop -- ROOM-NODE CONTROL PATTERN port (Path A2).
 *
 * Architecture (lifted from esp-openclaw-room-node, transport-agnostic):
 *   - ONE audio-owner task ("voice_owner") is the ONLY code that ever calls
 *     M5.Mic.begin/end, M5.Speaker.begin/end, playRaw, or touches the shared
 *     I2S port / codec I2C. Nothing else. Ever.
 *   - Every public entry point that changes hardware state POSTS a command to a
 *     FreeRTOS queue. Synchronous API calls (play_pcm, stream_begin/end, abort,
 *     selftest) attach a completion semaphore and block until the owner (or the
 *     playback pump) signals done. Fire-and-forget calls (start/stop listen,
 *     set_speaking) just post.
 *   - Explicit state machine: IDLE -> LISTENING -> THINKING -> SPEAKING -> IDLE.
 *     ALL transitions happen inside the owner task.
 *   - Barge-in (CMD_ABORT / a PTT press during SPEAKING) is a serialized
 *     SPEAKING->IDLE->LISTENING teardown INSIDE the owner -- never a concurrent
 *     free from two tasks. Combined with permanent (never-freed) ring/pool
 *     buffers, the heap double-free bug class is structurally impossible.
 *   - One bus mutex guards every I2S + shared-I2C (mic/speaker begin/end)
 *     touch as defense-in-depth (the owner is already the single toucher).
 *
 * The public API in stackchan_voice.h is FROZEN: every signature + observable
 * behavior is identical to before, so stackchan_voice_net.c (the ElevenLabs
 * STT/TTS transport) needs zero changes. This file only rewrote the internals.
 */

#include "stackchan_voice.h"
#include "stackchan_voice_net.h"
#include "stackchan_body.h"
#include "stackchan_idle.h" /* note activity -> reset idle timer / wake */
#include "esp_openclaw_node_stackchan_rgb_cmd.h" /* g_stackchan_power_poll_pause */

#include <M5Unified.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"

static const char *TAG = "stackchan_voice";

/* ---- Audio format (both directions, per spec) ---- */
static constexpr uint32_t VOICE_SAMPLE_RATE = 16000; /* PCM 16 kHz mono */
static constexpr size_t   REC_CHUNK_SAMPLES = 1600;  /* 100 ms per mic chunk */

/* ---- Streaming-playback tuning (unchanged from the proven half-duplex path) --- */
#define STREAM_RING_BYTES      (256 * 1024) /* ~8 s @ 16 kHz mono s16 */
#define STREAM_PREBUFFER_BYTES ( 32 * 1024) /* ~1 s banked before first sound */
#define STREAM_PLAY_CHUNK      ( 16 * 1024) /* ~512 ms per playRaw => few seams */
#define STREAM_CHANNEL          0           /* single channel => gapless queue */
#define STREAM_POOL             4           /* rotating play buffers (M5 keeps 2 in flight) */

/* Mic capture ring (canonical M5 lag-2 double-buffer; see capture step below). */
#define MIC_NBUF 4
#define MIC_LAG  2

/* ---- LED-ring feedback colors (RGB888) ---- */
static void led_idle(void)      { stackchan_led_fill(0, 0, 0);   } /* off (READY) */
static void led_listening(void) { stackchan_led_fill(0, 40, 0);  } /* green */
static void led_speaking(void)  { stackchan_led_fill(0, 0, 40);  } /* blue */

/* ================================================================= */
/* State machine (owner-task-private unless noted)                    */
/* ================================================================= */

typedef enum {
    VOICE_IDLE = 0,
    VOICE_LISTENING,
    VOICE_THINKING,
    VOICE_SPEAKING,
} voice_state_t;

static const char *state_name(voice_state_t s)
{
    switch (s) {
        case VOICE_IDLE:      return "IDLE";
        case VOICE_LISTENING: return "LISTENING";
        case VOICE_THINKING:  return "THINKING";
        case VOICE_SPEAKING:  return "SPEAKING";
        default:              return "?";
    }
}

static volatile voice_state_t s_state = VOICE_IDLE; /* written only by owner */

/* Cross-task intent flags (small, single-writer where it matters).
 *  - s_listen_intent: set synchronously by toggle/on/off so is_listening()
 *    reflects the caller's request instantly; owner reconciles the hardware.
 *  - s_stream_active: owned by the owner; read by is_streaming() and feed().
 *  - s_abort_req: fast barge-in flag set by the public abort/toggle BEFORE the
 *    CMD_ABORT is posted, so the playback pump can bail out of a playRaw retry
 *    immediately instead of waiting for the owner to dequeue the command. */
static volatile bool s_started      = false;
static volatile bool s_listen_intent = false;
static volatile bool s_stream_active = false;
static volatile bool s_abort_req     = false;

/* Speaker DMA persistence (see the big note above owner_speaker_ensure_open):
 * s_spk_persist = the I2S DMA is currently begun + HELD; s_spk_persist_rate =
 * the rate the held channel is configured for. Declared here (ahead of
 * owner_mic_begin, which clears the flag when the mic takes the shared port). */
static bool     s_spk_persist      = false;
static uint32_t s_spk_persist_rate = 0;

/* Confirm-gated loading state: on PTT-off we ARM (neutral LED, no I2C-hammering
 * animation yet); the pink "thinking"/loading LED lights ONLY when the gateway
 * confirms it received our turn (stackchan_voice_on_gateway_activity(), called
 * by net.c once it has an STT transcript to inject). s_await_deadline bounds the
 * arm so a late/stray confirm can't light the loading state long after the turn.
 * This both (a) implements the confirm-gated feature and (b) removes the
 * PTT-off I2C storm that tripped the AXP2101 false-shutdown, because the LED
 * animation now starts AFTER M5.Mic.end() and only on a real gateway signal. */
static volatile bool       s_await_reply    = false;
static volatile TickType_t s_await_deadline = 0;
#define AWAIT_REPLY_TIMEOUT_MS 20000

/* Diagnostics (owner-written). */
static volatile uint32_t s_utt_samples = 0;

/* Optional sinks for the network transport (mic PCM up + PTT edges). */
static stackchan_voice_mic_chunk_cb_t s_mic_cb = NULL;
static void *s_mic_cb_ctx = NULL;
static stackchan_voice_ptt_cb_t s_ptt_cb = NULL;
static void *s_ptt_cb_ctx = NULL;

/* ================================================================= */
/* Command queue + owner task                                         */
/* ================================================================= */

typedef enum {
    CMD_START_LISTEN = 0,
    CMD_STOP_LISTEN,
    CMD_PLAY_PCM,
    CMD_STREAM_BEGIN,
    CMD_STREAM_END,
    CMD_ABORT,
    CMD_SELFTEST,
    CMD_SET_SPEAKING,
    CMD_THINKING_ON,   /* light the confirm-gated loading LED (owner-serialized) */
    CMD_E2E_ARM,       /* E2E test: enter neutral THINKING wait (no mic, no LED) */
} cmd_type_t;

typedef struct {
    cmd_type_t type;
    const int16_t *pcm;      /* CMD_PLAY_PCM */
    size_t         samples;  /* CMD_PLAY_PCM */
    uint32_t       rate;     /* CMD_PLAY_PCM / CMD_STREAM_BEGIN */
    uint8_t        volume;   /* CMD_STREAM_BEGIN */
    bool           flag;     /* CMD_SET_SPEAKING */
    SemaphoreHandle_t done;  /* optional completion sema */
    esp_err_t     *result;   /* optional result out */
} voice_cmd_t;

static QueueHandle_t     s_cmd_q   = NULL;
static TaskHandle_t      s_owner   = NULL;
static SemaphoreHandle_t s_bus_mtx = NULL;   /* guards all I2S + codec-I2C begin/end */

/* BUS_LOCK also PAUSES the AXP2101 power-key poller for the duration of the
 * lock. The poller and the M5Unified audio codecs share the CoreS3 internal
 * I2C bus through two separate driver stacks; pausing the poller while we do
 * codec begin/end I2C prevents the cross-stack collision that made
 * M5.Speaker.begin() fail (reply never plays) and misfired the false
 * "power key -> graceful shutdown". Recursive-lock safe: the counter is
 * incremented on every (recursive) take and decremented on every give, so it
 * only reaches 0 when the outermost lock is released. Written only by the
 * owner task; the poller reads it. */
#define BUS_LOCK()   do { if (s_bus_mtx) xSemaphoreTakeRecursive(s_bus_mtx, portMAX_DELAY); g_stackchan_power_poll_pause++; } while (0)
#define BUS_UNLOCK() do { if (g_stackchan_power_poll_pause > 0) { g_stackchan_power_poll_pause--; } if (s_bus_mtx) xSemaphoreGiveRecursive(s_bus_mtx); } while (0)

/* Permanent audio buffers (allocated ONCE at start, NEVER freed while running).
 * Because they are never freed, no barge-in race can double-free them -- the
 * entire crash class the port targets is removed by construction, not patched. */
static int16_t *s_mic_ring[MIC_NBUF] = { NULL, NULL, NULL, NULL };
static StreamBufferHandle_t s_stream_sb = NULL;
static StaticStreamBuffer_t s_stream_sb_struct;
static uint8_t *s_stream_storage = NULL;
static int16_t *s_play_pool[STREAM_POOL] = { NULL, NULL, NULL, NULL };

/* Streaming pump state (owner-private). */
static uint32_t s_stream_rate    = 16000;
static volatile bool s_stream_eof = false;
static bool     s_stream_started = false; /* prebuffer satisfied? */
static int      s_play_slot      = 0;

/* A deferred stream_end() waiter: the owner stashes the completion here and the
 * playback pump (or an abort) signals it once the DAC has actually drained.
 * This is what makes "barge-in during normal stream_end" safe -- both paths
 * converge on the single owner and exactly one teardown fires. */
static SemaphoreHandle_t s_pending_end_done   = NULL;
static esp_err_t        *s_pending_end_result = NULL;

/* Mic capture ring cursor (owner-private, valid while LISTENING). */
static uint32_t s_mic_head    = 0;
static bool     s_mic_primed  = false;
static uint32_t s_mic_chunk   = 0;

static void cmd_complete(voice_cmd_t *c, esp_err_t err)
{
    if (c->result) { *c->result = err; }
    if (c->done)   { xSemaphoreGive(c->done); }
}

/* Post a command; if wait_ticks>0, block on a completion sema and return the
 * owner-supplied result. */
static esp_err_t voice_post(cmd_type_t type, const int16_t *pcm, size_t samples,
                            uint32_t rate, uint8_t volume, bool flag,
                            TickType_t wait_ticks)
{
    voice_cmd_t cmd = {};
    cmd.type = type;
    cmd.pcm = pcm;
    cmd.samples = samples;
    cmd.rate = rate;
    cmd.volume = volume;
    cmd.flag = flag;

    esp_err_t result = ESP_OK;
    SemaphoreHandle_t done = NULL;
    if (wait_ticks > 0) {
        done = xSemaphoreCreateBinary();
        if (done == NULL) { return ESP_ERR_NO_MEM; }
        cmd.done = done;
        cmd.result = &result;
    }
    if (s_cmd_q == NULL ||
        xQueueSend(s_cmd_q, &cmd, pdMS_TO_TICKS(1000)) != pdTRUE) {
        if (done) { vSemaphoreDelete(done); }
        return ESP_ERR_TIMEOUT;
    }
    if (done) {
        if (xSemaphoreTake(done, wait_ticks) != pdTRUE) {
            /* Timed out waiting for the owner. Leave the sema leaked-free:
             * we cannot delete it safely if the owner might still signal it.
             * In practice this path means a wedged owner (should not happen). */
            ESP_LOGW(TAG, "cmd %d timed out waiting for owner", (int)type);
            return ESP_ERR_TIMEOUT;
        }
        vSemaphoreDelete(done);
    }
    return result;
}

/* Compute RMS of a PCM chunk (0..32767-ish). */
static float pcm_rms(const int16_t *pcm, size_t n)
{
    if (n == 0) { return 0.0f; }
    double acc = 0.0;
    for (size_t i = 0; i < n; i++) {
        double s = (double)pcm[i];
        acc += s * s;
    }
    return (float)sqrt(acc / (double)n);
}

/* ----------------------------------------------------------------- */
/* Owner-only hardware helpers                                        */
/* ----------------------------------------------------------------- */

/* Bring the mic up for a listening window. Ends the speaker first (half-duplex
 * shared I2S). Returns false on begin() failure. */
static bool owner_mic_begin(void)
{
    BUS_LOCK();
    if (M5.Speaker.isEnabled()) {
        /* Half-duplex: the mic needs the shared I2S_NUM_1 port, so this is the
         * ONE place we truly free the speaker DMA. s_spk_persist is cleared so
         * the next stream re-opens (owner_speaker_ensure_open re-allocs). This
         * path is dormant in the current reply-audio (playStream) config. */
        M5.Speaker.end();
        s_spk_persist = false;
    }
    /* Clean RAW audio to the transcriber: ElevenLabs runs its own AGC/denoise,
     * so we do NOT overdrive or pre-process. Default 16x mag clips speech peaks
     * (and amplifies the noise floor) -- drop it and disable the on-chip filter
     * so the provider gets the true wideband signal. */
    {
        auto mcfg = M5.Mic.config();
        mcfg.sample_rate = VOICE_SAMPLE_RATE;
        mcfg.magnification = 4;      /* was default 16 -> clipping */
        mcfg.noise_filter_level = 0; /* let ElevenLabs denoise */
        mcfg.over_sampling = 2;
        M5.Mic.config(mcfg);
    }
    bool ok = M5.Mic.begin();
    if (ok) {
        M5.Mic.setSampleRate(VOICE_SAMPLE_RATE);
    }
    BUS_UNLOCK();
    return ok;
}

static void owner_mic_end(void)
{
    /* Wait out any in-flight DMA fill so the background task isn't writing into
     * a buffer as we tear the mic down. */
    while (M5.Mic.isRecording()) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    BUS_LOCK();
    if (M5.Mic.isEnabled()) {
        M5.Mic.end();
    }
    BUS_UNLOCK();
}

/* ================================================================= */
/* Speaker DMA persistence (queue item #3)                            */
/* ================================================================= */
/* The I2S DMA that Speaker.begin() allocates comes from the SCARCE internal
 * DMA-capable heap (MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA). The old design did
 * Speaker.begin() (alloc ~4096 B contiguous) at the START of EVERY reply stream
 * and Speaker.end() (free) at the end. That per-turn alloc/free churned and
 * re-fragmented the internal DMA pool over a session, so after several replies
 * the largest contiguous free block sawtoothed down toward the exact ~4096 B
 * needed (seen as tight as 4096 vs 4096 = zero margin) and could intermittently
 * lose the fragmentation race -> Speaker.begin() fails -> gateway retries ->
 * delayed/dropped reply audio (the residual of the original intermittent-audio
 * pain).
 *
 * FIX: claim the I2S DMA ONCE, at boot, while the internal heap is still
 * contiguous (stackchan_voice_start() runs BEFORE Wi-Fi/TLS/WireGuard fragment
 * internal RAM), and HOLD it for the life of the device. A per-turn stream then
 * just reuses the already-open channel (write samples), never re-allocating.
 * The ~4096 B DMA is claimed once and never returned, so it can never lose the
 * fragmentation race mid-session.
 *
 * Shared-I2S note: on CoreS3 the mic (ES7210) and speaker (AW88298) share
 * I2S_NUM_1 (half-duplex), so owner_mic_begin() still tears the speaker down
 * (frees the DMA) when it needs the port; the next stream re-opens it then.
 * That mic/PTT path is dormant in the current gateway config (replies arrive
 * via speaker.playStream, not talk.speak), so in normal operation the held DMA
 * is never returned and the sawtooth is eliminated. The AW88298 sample-rate
 * register is programmed in the M5 enable callback from spk_cfg.sample_rate at
 * begin() time, so the held channel keeps a FIXED output rate and M5's mixer
 * resamples each reply to it; a full config+begin (realloc) happens only when
 * the speaker is closed (boot, or after the mic took the port) or when the
 * source rate actually changes (the gateway TTS rate is stable, so this is a
 * one-time event at most). */

/* Set to 1 to emit per-turn SPKDMA headroom telemetry (proves the held block is
 * stable / no per-turn alloc-free). Off by default; flip to 1 to re-verify. */
#define SPK_DMA_DEBUG 0

/* Boot-claim rate: measured on-device, the gateway's reply-audio (playStream)
 * arrives as pcm_24000, so claim the held channel at 24 kHz -> the FIRST real
 * reply reuses the held DMA with zero re-alloc (a different rate self-adapts via
 * a one-time re-open in owner_speaker_ensure_open). */
#define SPK_PERSIST_DEFAULT_RATE 24000

/* Ensure the speaker I2S channel + DMA are up and configured for `rate`, then
 * (re)set the volume. If the channel is ALREADY open at the same rate this is a
 * no-op alloc-wise -- it reuses the HELD DMA (the persistence that kills the
 * per-turn re-fragmentation). A full config+begin (DMA alloc) happens only when
 * the channel is closed (boot / post-mic) or the source rate changed. DMA sized
 * to ~256 ms; mixer pinned to core 1. */
static bool owner_speaker_ensure_open(uint32_t rate, uint8_t volume)
{
    BUS_LOCK();
    if (M5.Speaker.isEnabled() && s_spk_persist && s_spk_persist_rate == rate) {
        /* Reuse the held DMA -- NO i2s_new_channel, NO fragmentation. */
        M5.Speaker.setVolume(volume);
#if SPK_DMA_DEBUG
        ESP_LOGI(TAG, "SPKDMA reuse (held) @ %u Hz largest-dma=%u free-dma=%u",
                 (unsigned)rate,
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
#endif
        BUS_UNLOCK();
        return true;
    }
    /* (Re)allocation path: closed, or the rate changed. Tear down any stale
     * channel first so we reconfigure cleanly. */
    if (M5.Speaker.isEnabled()) {
        M5.Speaker.end();
    }
    s_spk_persist = false;
    {
        auto scfg = M5.Speaker.config();
        scfg.sample_rate = rate;
        /* 256x8 (~4 KB total, 512-byte blocks): fits under the clean internal
         * DMA floor AND the small blocks survive fragmentation. 8 buffers of
         * 256 frames still give ~85 ms of ring @ 24 kHz for gapless mixing. */
        scfg.dma_buf_len = 256;
        scfg.dma_buf_count = 8;
        scfg.task_pinned_core = 1;
        scfg.task_priority = 10;
        M5.Speaker.config(scfg);
    }
    const unsigned scfg_dma_len = 256;
    const unsigned scfg_dma_cnt = 8;
    /* Retry a transient codec-init NAK: the poller is paused (BUS_LOCK) so a
     * retry lands on a quiet bus. A single stray i2c collision must not lose
     * the whole reply. */
    bool ok = false;
    ESP_LOGI(TAG, "SPKDMA (re)alloc @ %u Hz: free-dma=%u largest-dma=%u (need ~%u)",
             (unsigned)rate,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
             (unsigned)(scfg_dma_len * scfg_dma_cnt * 2));
    for (int attempt = 0; attempt < 4 && !ok; attempt++) {
        if (attempt > 0) {
            M5.Speaker.end();
            vTaskDelay(pdMS_TO_TICKS(30));
            ESP_LOGW(TAG, "Speaker.begin retry %d (stream) [free internal=%u dma=%u largest-dma=%u]",
                     attempt,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
        }
        ok = M5.Speaker.begin();
    }
    if (ok) {
        M5.Speaker.setVolume(volume);
        s_spk_persist = true;
        s_spk_persist_rate = rate;
    }
    BUS_UNLOCK();
    return ok;
}

/* Start the speaker for a one-shot block if it is not already up. */
static bool owner_speaker_begin_oneshot(void)
{
    BUS_LOCK();
    bool ok = true;
    if (!M5.Speaker.isEnabled()) {
        for (int attempt = 0; attempt < 4; attempt++) {
            if (attempt > 0) {
                M5.Speaker.end();
                vTaskDelay(pdMS_TO_TICKS(30));
                ESP_LOGW(TAG, "Speaker.begin retry %d (oneshot)", attempt);
            }
            ok = M5.Speaker.begin();
            if (ok) { break; }
        }
        if (ok) { M5.Speaker.setVolume(160); }
    }
    BUS_UNLOCK();
    return ok;
}

/* End-of-reply teardown that KEEPS the DMA (persistence). Just make sure no
 * audio is queued so the next reply starts clean, WITHOUT i2s_del_channel --
 * the held I2S DMA stays allocated for the next stream. M5.Speaker.stop() clears
 * queued wavs; spk_task then feeds silence (auto_clear DMA) and sleeps, so there
 * is no trailing buzz between replies. Contrast with M5.Speaker.end() (frees the
 * DMA) which we now call ONLY when the mic takes the shared I2S port. */
static void owner_speaker_idle(void)
{
    BUS_LOCK();
    if (M5.Speaker.isEnabled()) {
        M5.Speaker.stop();
    }
    BUS_UNLOCK();
}

/* Drop any bytes lingering in the ring from a previous (possibly aborted)
 * stream so a fresh stream never plays stale audio. Owner-only. */
static void owner_stream_drain_stale(void)
{
    if (s_stream_sb == NULL) { return; }
    static uint8_t scratch[2048];
    while (xStreamBufferReceive(s_stream_sb, scratch, sizeof(scratch), 0) > 0) {
        /* discard */
    }
}

/* Finish an active stream -- either a clean end (drained) or a barge-in abort.
 * Runs ONLY in the owner. Signals any deferred stream_end() waiter exactly
 * once. Never frees the (permanent) ring/pool. */
static void owner_stream_finish(bool aborted)
{
    if (aborted) {
        M5.Speaker.stop(); /* dump queued DAC audio so sound stops now */
    } else {
        while (M5.Speaker.isPlaying()) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    s_stream_active = false;
    s_stream_eof = true;
    s_stream_started = false;
    owner_speaker_idle(); /* stop playback but HOLD the DMA (persistence) */
    stackchan_voice_set_speaking(false);
    s_state = VOICE_IDLE;
    printf("STATE -> IDLE\n"); /* E2E scrapeable state token */
    if (s_pending_end_done != NULL) {
        if (s_pending_end_result) { *s_pending_end_result = ESP_OK; }
        xSemaphoreGive(s_pending_end_done);
        s_pending_end_done = NULL;
        s_pending_end_result = NULL;
    }
    ESP_LOGI(TAG, "stream %s -> IDLE", aborted ? "ABORT" : "end");
}

/* ----------------------------------------------------------------- */
/* Owner: per-loop state work                                         */
/* ----------------------------------------------------------------- */

static void owner_enter_listen(void)
{
    s_await_reply = false;         /* re-listening: cancel any pending confirm-gate */
    stackchan_thinking_set(false); /* mic reopened: cancel any waiting state */
    if (!owner_mic_begin()) {
        ESP_LOGE(TAG, "M5.Mic.begin() failed; aborting capture");
        s_listen_intent = false;
        s_state = VOICE_IDLE;
        led_idle();
        return;
    }
    /* Prime LAG buffers so the buffer we consume (head-LAG) is always full. */
    s_mic_head = 0;
    s_mic_primed = false;
    s_mic_chunk = 0;
    s_utt_samples = 0;
    for (int i = 0; i < MIC_LAG; i++) {
        M5.Mic.record(s_mic_ring[s_mic_head % MIC_NBUF], REC_CHUNK_SAMPLES,
                      VOICE_SAMPLE_RATE, false);
        s_mic_head++;
    }
    s_state = VOICE_LISTENING;
    led_listening();
    ESP_LOGI(TAG, "mic capture START (%u Hz mono, %u-sample chunks)",
             (unsigned)VOICE_SAMPLE_RATE, (unsigned)REC_CHUNK_SAMPLES);
}

/* One mic capture cycle. record() paces the loop (blocks until a DMA slot frees)
 * so we never poll isRecording() to detect fill -- that async race doubled every
 * chunk in the old code. Consume the buffer that lags the producer by MIC_LAG. */
static void owner_capture_step(void)
{
    if (!M5.Mic.record(s_mic_ring[s_mic_head % MIC_NBUF], REC_CHUNK_SAMPLES,
                       VOICE_SAMPLE_RATE, false)) {
        vTaskDelay(pdMS_TO_TICKS(2));
        return;
    }
    int16_t *ready = s_mic_ring[(s_mic_head - MIC_LAG) % MIC_NBUF];
    s_mic_head++;

    if (!s_mic_primed) { /* drop first chunk (DC settling transient) */
        s_mic_primed = true;
        return;
    }
    s_utt_samples += REC_CHUNK_SAMPLES;
    if ((s_mic_chunk % 10) == 0) {
        float rms = pcm_rms(ready, REC_CHUNK_SAMPLES);
        ESP_LOGI(TAG, "  mic chunk %u rms=%.1f (%.1f s captured)",
                 (unsigned)s_mic_chunk, rms,
                 (double)s_utt_samples / (double)VOICE_SAMPLE_RATE);
    }
    s_mic_chunk++;
    if (s_mic_cb != NULL) {
        s_mic_cb(ready, REC_CHUNK_SAMPLES, s_mic_cb_ctx);
    }
}

static void owner_stop_listen(void)
{
    owner_mic_end();
    /* Neutral "armed/waiting" look -- NOT the pink thinking animation. The pink
     * loading LED lights only once the gateway confirms (CMD_THINKING_ON). This
     * single LED write happens after M5.Mic.end() so it can't collide with the
     * codec I2C. */
    led_idle();
    ESP_LOGI(TAG, "mic capture STOP: %u chunks, %.2f s total",
             (unsigned)s_mic_chunk,
             (double)s_utt_samples / (double)VOICE_SAMPLE_RATE);
    s_state = VOICE_THINKING; /* waiting for the reply; loading LED gated on confirm */
}

/* One streaming-playback cycle: prebuffer gate, then drain a chunk into the
 * speaker; detect drain-complete on EOF. Barge-in (s_abort_req) short-circuits
 * so the pending CMD_ABORT is serviced on the next loop iteration. */
static void owner_pump_step(void)
{
    if (s_abort_req) { return; }
    if (!s_stream_started) {
        size_t buffered = STREAM_RING_BYTES - xStreamBufferSpacesAvailable(s_stream_sb);
        if (s_stream_eof || buffered >= STREAM_PREBUFFER_BYTES) {
            s_stream_started = true;
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
            return;
        }
    }
    int16_t *chunk = s_play_pool[s_play_slot];
    size_t n = xStreamBufferReceive(s_stream_sb, chunk, STREAM_PLAY_CHUNK,
                                    pdMS_TO_TICKS(20));
    if (n > 0) {
        size_t samples = n / sizeof(int16_t);
        for (int attempt = 0; attempt < 4000; attempt++) {
            if (s_abort_req) { return; }
            if (M5.Speaker.playRaw(chunk, samples, s_stream_rate,
                                   false, 1, STREAM_CHANNEL, false)) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        s_play_slot = (s_play_slot + 1) % STREAM_POOL;
        return;
    }
    /* Ring empty: are we done? */
    if (s_stream_eof && xStreamBufferIsEmpty(s_stream_sb) && !M5.Speaker.isPlaying()) {
        owner_stream_finish(false);
    }
}

/* ----------------------------------------------------------------- */
/* Owner: command handlers                                            */
/* ----------------------------------------------------------------- */

static void owner_handle_cmd(voice_cmd_t *cmd)
{
    switch (cmd->type) {
    case CMD_START_LISTEN:
        if (s_state == VOICE_SPEAKING || s_stream_active) {
            owner_stream_finish(true); /* barge-in: tear playback down first */
        }
        if (s_state != VOICE_LISTENING) {
            owner_enter_listen();
        }
        cmd_complete(cmd, ESP_OK);
        break;

    case CMD_STOP_LISTEN:
        if (s_state == VOICE_LISTENING) {
            owner_stop_listen();
        }
        cmd_complete(cmd, ESP_OK);
        break;

    case CMD_PLAY_PCM: {
        s_await_reply = false;
        stackchan_thinking_set(false);
        if (s_state == VOICE_LISTENING) {
            owner_mic_end();
        }
        s_state = VOICE_SPEAKING;
        printf("STATE -> SPEAKING\n"); /* E2E scrapeable state token */
        esp_err_t err = ESP_OK;
        if (!owner_speaker_begin_oneshot()) {
            ESP_LOGE(TAG, "M5.Speaker.begin() failed (play_pcm)");
            err = ESP_FAIL;
        } else {
            bool played = M5.Speaker.playRaw(cmd->pcm, cmd->samples, cmd->rate, false, 1, -1);
            while (M5.Speaker.isPlaying()) {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            err = played ? ESP_OK : ESP_FAIL;
        }
        /* Leave the speaker enabled between back-to-back TTS chunks (matches the
         * proven behavior; a later listen/stream ends it). */
        s_state = VOICE_IDLE;
        printf("STATE -> IDLE\n"); /* E2E scrapeable state token */
        cmd_complete(cmd, err);
        break;
    }

    case CMD_SELFTEST: {
        stackchan_thinking_set(false);
        if (s_state == VOICE_LISTENING) {
            owner_mic_end();
        }
        s_state = VOICE_SPEAKING;
        esp_err_t err = ESP_OK;
        if (!owner_speaker_begin_oneshot()) {
            err = ESP_FAIL;
        } else {
            bool played = M5.Speaker.playRaw(cmd->pcm, cmd->samples, cmd->rate, false, 1, -1);
            while (M5.Speaker.isPlaying()) {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            err = played ? ESP_OK : ESP_FAIL;
        }
        owner_speaker_idle(); /* keep the held DMA */
        stackchan_voice_set_speaking(false);
        s_state = VOICE_IDLE;
        cmd_complete(cmd, err);
        break;
    }

    case CMD_STREAM_BEGIN: {
        s_await_reply = false;
        stackchan_thinking_set(false);
        s_listen_intent = false;
        if (s_state == VOICE_LISTENING) {
            owner_mic_end();
        }
        /* If a stream is somehow still active, finish it first (no double-free:
         * permanent buffers, single owner). */
        if (s_stream_active) {
            owner_stream_finish(true);
        }
        owner_stream_drain_stale();
        s_abort_req = false;
#if SPK_DMA_DEBUG
        /* Per-turn headroom snapshot (proves the held block is stable across
         * turns / not sawtoothing down toward 4096). */
        ESP_LOGI(TAG, "SPKDMA turn: largest-dma=%u free-dma=%u persist=%d rate=%u",
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
                 (int)s_spk_persist, (unsigned)cmd->rate);
#endif
        esp_err_t err = ESP_OK;
        if (!owner_speaker_ensure_open(cmd->rate, cmd->volume)) {
            ESP_LOGE(TAG, "M5.Speaker.begin() failed (stream)");
            s_state = VOICE_IDLE;
            err = ESP_FAIL;
        } else {
            s_stream_rate = cmd->rate;
            s_stream_eof = false;
            s_stream_started = false;
            s_play_slot = 0;
            s_stream_active = true;
            s_state = VOICE_SPEAKING;
            printf("STATE -> SPEAKING\n"); /* E2E scrapeable state token (streamed TTS) */
            stackchan_voice_set_speaking(true);
            ESP_LOGI(TAG, "stream begin @ %u Hz vol=%u",
                     (unsigned)cmd->rate, (unsigned)cmd->volume);
        }
        cmd_complete(cmd, err);
        break;
    }

    case CMD_STREAM_END:
        if (!s_stream_active) {
            /* Already torn down (e.g. a barge-in abort beat us here) -> no-op.
             * This is the old double-free path, now a trivial safe return. */
            cmd_complete(cmd, ESP_OK);
        } else {
            /* Defer completion to the pump: mark EOF and stash the waiter. The
             * pump signals it once the DAC has actually drained. */
            s_stream_eof = true;
            s_pending_end_done = cmd->done;
            s_pending_end_result = cmd->result;
            /* Do NOT cmd_complete here. */
        }
        break;

    case CMD_ABORT:
        s_abort_req = false;
        if (s_stream_active || s_state == VOICE_SPEAKING) {
            owner_stream_finish(true); /* also signals any deferred stream_end */
        }
        cmd_complete(cmd, ESP_OK);
        break;

    case CMD_SET_SPEAKING:
        if (cmd->flag) { led_speaking(); }
        else           { led_idle(); }
        cmd_complete(cmd, ESP_OK);
        break;

    case CMD_THINKING_ON:
        /* Confirm-gated loading state: the gateway acknowledged our turn while
         * we were armed and still waiting (state THINKING). Light the pink
         * loading LED now -- serialized on the owner so thinking_set()'s task
         * creation + 120ms settle never runs on net.c's timer daemon, and so
         * it happens with the mic already torn down (no codec/LED I2C clash). */
        if (s_await_reply && s_state == VOICE_THINKING) {
            s_await_reply = false;
            ESP_LOGI(TAG, "gateway confirmed turn received -> loading/thinking LED on");
            printf("STATE -> THINKING\n"); /* E2E: confirm-gated -- only after gateway transcript */
            stackchan_thinking_set(true);
        }
        cmd_complete(cmd, ESP_OK);
        break;

    case CMD_E2E_ARM:
        /* E2E test only: enter the SAME neutral "armed/waiting" wait a real
         * PTT-off produces (state THINKING, NO loading LED yet), but without a
         * physical mic to tear down. The pink loading LED still lights only via
         * the confirm-gated CMD_THINKING_ON path once the gateway returns a
         * transcript, so this faithfully exercises the confirm gate. */
        led_idle();
        s_state = VOICE_THINKING;
        cmd_complete(cmd, ESP_OK);
        break;

    default:
        cmd_complete(cmd, ESP_ERR_INVALID_ARG);
        break;
    }
}

static void voice_owner_task(void *arg)
{
    (void)arg;
    for (;;) {
        /* IDLE/THINKING: block on the queue. LISTENING/SPEAKING: poll the queue
         * (wait 0) so we stay responsive, then do one unit of audio work. */
        TickType_t wait = (s_state == VOICE_LISTENING || s_state == VOICE_SPEAKING)
                              ? 0 : portMAX_DELAY;
        voice_cmd_t cmd;
        if (xQueueReceive(s_cmd_q, &cmd, wait) == pdTRUE) {
            owner_handle_cmd(&cmd);
        }
        if (s_state == VOICE_LISTENING) {
            owner_capture_step();
        } else if (s_state == VOICE_SPEAKING && s_stream_active) {
            owner_pump_step();
        }
    }
}

/* ================================================================= */
/* Public API (FROZEN seam)                                           */
/* ================================================================= */

void stackchan_voice_set_mic_chunk_cb(stackchan_voice_mic_chunk_cb_t cb, void *ctx)
{
    s_mic_cb = cb;
    s_mic_cb_ctx = ctx;
}

void stackchan_voice_set_ptt_cb(stackchan_voice_ptt_cb_t cb, void *ctx)
{
    s_ptt_cb = cb;
    s_ptt_cb_ctx = ctx;
}

static bool alloc_permanent_buffers(void)
{
    for (int i = 0; i < MIC_NBUF; i++) {
        s_mic_ring[i] = (int16_t *)heap_caps_malloc(
            REC_CHUNK_SAMPLES * sizeof(int16_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
        if (s_mic_ring[i] == NULL) { return false; }
    }
    /* Stream ring in PSRAM (fall back to internal). */
    s_stream_storage = (uint8_t *)heap_caps_malloc(
        STREAM_RING_BYTES + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_stream_storage == NULL) {
        s_stream_storage = (uint8_t *)heap_caps_malloc(STREAM_RING_BYTES + 1, MALLOC_CAP_8BIT);
    }
    if (s_stream_storage == NULL) { return false; }
    s_stream_sb = xStreamBufferCreateStatic(
        STREAM_RING_BYTES, 1, s_stream_storage, &s_stream_sb_struct);
    if (s_stream_sb == NULL) { return false; }
    /* Rotating play pool in PSRAM: M5.Speaker.playRaw streams directly from the
     * caller buffer and holds up to 2 waveforms in flight, so a chunk must not
     * be reused until M5 is done -- 4 slots > 2 in-flight + 1 filling. */
    for (int i = 0; i < STREAM_POOL; i++) {
        s_play_pool[i] = (int16_t *)heap_caps_malloc(STREAM_PLAY_CHUNK,
                                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_play_pool[i] == NULL) {
            s_play_pool[i] = (int16_t *)heap_caps_malloc(STREAM_PLAY_CHUNK, MALLOC_CAP_8BIT);
        }
        if (s_play_pool[i] == NULL) { return false; }
    }
    return true;
}

esp_err_t stackchan_voice_start(void)
{
    if (s_started) {
        return ESP_OK;
    }
    s_bus_mtx = xSemaphoreCreateRecursiveMutex();
    s_cmd_q = xQueueCreate(16, sizeof(voice_cmd_t));
    if (s_bus_mtx == NULL || s_cmd_q == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (!alloc_permanent_buffers()) {
        ESP_LOGE(TAG, "permanent audio buffer alloc failed");
        return ESP_ERR_NO_MEM;
    }

    /* Speaker DMA persistence: claim the ~4 KB internal I2S DMA ONCE, now, while
     * the internal heap is still contiguous. stackchan_voice_start() runs at
     * boot BEFORE Wi-Fi/TLS/WireGuard fragment internal RAM, so this is the
     * best moment to grab the contiguous block and HOLD it for the device's
     * life -- eliminating the per-turn begin/end that re-fragmented the pool and
     * lost the ~4096-vs-4096 race after several replies. Safe here: no other
     * task touches the shared bus yet (power poller starts later in app_main),
     * and the owner task is created just below. Non-fatal on failure: the stream
     * path will lazily (re)open per reply as before. */
    if (owner_speaker_ensure_open(SPK_PERSIST_DEFAULT_RATE, 160)) {
        ESP_LOGI(TAG, "speaker DMA persistence: claimed I2S DMA once at boot @ %u Hz (largest-dma now=%u)",
                 (unsigned)SPK_PERSIST_DEFAULT_RATE,
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    } else {
        ESP_LOGW(TAG, "speaker DMA persistence: boot begin failed; will lazily begin per-stream");
    }

    led_idle();
    s_state = VOICE_IDLE;

    BaseType_t ok = xTaskCreatePinnedToCore(
        voice_owner_task, "voice_owner", 6144, NULL, 6, &s_owner, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create voice owner task");
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    ESP_LOGI(TAG, "voice subsystem started (idle, owner-task control pattern)");
    return ESP_OK;
}

bool stackchan_voice_toggle_listening(void)
{
    if (!s_started) {
        stackchan_voice_start();
    }
    /* PTT/button is activity: reset the idle timers + wake from light idle. */
    stackchan_idle_note_activity();

    bool now;
    if (s_stream_active) {
        /* BARGE-IN: side button interrupts TTS and opens the mic. Set the fast
         * abort flag so the pump bails immediately, then serialize
         * abort->listen through the owner queue (no concurrent teardown). */
        s_abort_req = true;
        voice_post(CMD_ABORT, NULL, 0, 0, 0, false, 0);
        s_await_reply = false;
        stackchan_thinking_set(false);
        s_listen_intent = true;
        now = true;
        voice_post(CMD_START_LISTEN, NULL, 0, 0, 0, false, 0);
        ESP_LOGI(TAG, "=== PTT BARGE-IN: playback stopped, mic hot ===");
    } else {
        now = !s_listen_intent;
        s_listen_intent = now;
        if (now) {
            s_await_reply = false;
            stackchan_thinking_set(false);
            voice_post(CMD_START_LISTEN, NULL, 0, 0, 0, false, 0);
            ESP_LOGI(TAG, "=== PTT: LISTENING ON  (mic hot) ===");
        } else {
            /* Do NOT light the thinking LED here. Lighting it on PTT-off (before
             * ANY gateway confirmation) starts a ~25fps I2C-hammering LED task
             * that wedges the shared In_I2C bus across M5.Mic.end() and trips
             * the AXP2101 power poller into a false "power key -> graceful
             * shutdown" storm. Instead ARM: the pink loading LED lights only
             * once the gateway confirms it received our turn
             * (stackchan_voice_on_gateway_activity -> CMD_THINKING_ON), which
             * also happens after mic.end(). Bounded by s_await_deadline. */
            s_await_reply = true;
            s_await_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(AWAIT_REPLY_TIMEOUT_MS);
            voice_post(CMD_STOP_LISTEN, NULL, 0, 0, 0, false, 0);
            ESP_LOGI(TAG, "=== PTT: LISTENING OFF (end of utterance) -> armed, awaiting gateway confirm ===");
        }
    }

    /* Notify the network transport AFTER posting so its RPCs don't contend. */
    if (s_ptt_cb != NULL) {
        s_ptt_cb(now, s_ptt_cb_ctx);
    }
    return now;
}

bool stackchan_voice_is_listening(void)
{
    return s_listen_intent;
}

void stackchan_voice_on_gateway_activity(void)
{
    /* Confirm-gated loading state. Called by the network transport once the
     * gateway has confirmed our turn (an STT transcript is available to inject).
     * No-op unless we are armed (PTT released, awaiting a reply) and still in
     * the neutral THINKING wait; a reply already playing / a fresh listen clear
     * the arm elsewhere. Bounded by s_await_deadline so a stale/late confirm
     * can't light the loading state after the turn has moved on. The actual LED
     * task creation is deferred to the owner (CMD_THINKING_ON) so it never runs
     * on net.c's timer daemon and never clashes with codec I2C. */
    if (!s_started || !s_await_reply) {
        return;
    }
    if (s_state != VOICE_THINKING) {
        s_await_reply = false;
        return;
    }
    if ((int32_t)(xTaskGetTickCount() - s_await_deadline) >= 0) {
        s_await_reply = false; /* arm window elapsed -> stay idle */
        return;
    }
    voice_post(CMD_THINKING_ON, NULL, 0, 0, 0, false, 0);
}

void stackchan_voice_set_speaking(bool speaking)
{
    /* Called by the owner itself (direct) and by the network transport (posted).
     * When we ARE the owner task, update the LED directly to avoid self-deadlock
     * on the queue; otherwise post so the owner remains the single toucher. */
    if (xTaskGetCurrentTaskHandle() == s_owner || s_cmd_q == NULL) {
        if (speaking) { led_speaking(); } else { led_idle(); }
        return;
    }
    voice_post(CMD_SET_SPEAKING, NULL, 0, 0, 0, speaking, 0);
}

esp_err_t stackchan_voice_play_pcm(const int16_t *pcm, size_t samples, uint32_t sample_rate)
{
    if (pcm == NULL || samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_started) {
        stackchan_voice_start();
    }
    /* Synchronous: block until the owner has played the whole block so
     * back-to-back TTS chunks stay ordered. Generous ceiling for long clips. */
    return voice_post(CMD_PLAY_PCM, pcm, samples, sample_rate, 0, false,
                      pdMS_TO_TICKS(60000));
}

esp_err_t stackchan_voice_stream_begin(uint32_t sample_rate, uint8_t volume)
{
    if (sample_rate < 8000 || sample_rate > 48000) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_started) {
        stackchan_voice_start();
    }
    /* An incoming gateway turn's reply audio is activity: wake instantly so the
     * screen/servos/WiFi are restored while the (still-claimed) speaker DMA
     * plays the first reply cleanly. */
    stackchan_idle_note_activity();
    return voice_post(CMD_STREAM_BEGIN, NULL, 0, sample_rate, volume, false,
                      pdMS_TO_TICKS(5000));
}

esp_err_t stackchan_voice_stream_feed(const int16_t *pcm, size_t samples)
{
    if (pcm == NULL || samples == 0 || s_stream_sb == NULL || !s_stream_active) {
        return ESP_OK;
    }
    /* Feed writes DIRECTLY into the permanent stream buffer (safe single-writer;
     * the owner is the single reader). Blocking send => natural backpressure.
     * The buffer is never deleted, so there is no use-after-free on abort -- the
     * s_stream_active / s_abort_req checks just make a late feed a no-op. */
    size_t bytes = samples * sizeof(int16_t);
    const uint8_t *p = (const uint8_t *)pcm;
    while (bytes > 0 && s_stream_active && !s_abort_req) {
        size_t sent = xStreamBufferSend(s_stream_sb, p, bytes, pdMS_TO_TICKS(2000));
        if (sent == 0) {
            ESP_LOGW(TAG, "stream feed: ring stalled");
            return ESP_ERR_TIMEOUT;
        }
        p += sent;
        bytes -= sent;
    }
    return ESP_OK;
}

void stackchan_voice_stream_end(void)
{
    if (!s_started) {
        return;
    }
    /* Blocks until the pump drains the DAC (or an abort converges). Bounded so a
     * wedged pump can't hang the RPC forever (~35 s ceiling for long clips). */
    voice_post(CMD_STREAM_END, NULL, 0, 0, 0, false, pdMS_TO_TICKS(35000));
    ESP_LOGI(TAG, "stream end (returned)");
}

esp_err_t stackchan_voice_stream_abort(void)
{
    if (!s_started) {
        return ESP_OK;
    }
    s_abort_req = true; /* fast-path so the pump bails before the owner dequeues */
    return voice_post(CMD_ABORT, NULL, 0, 0, 0, false, pdMS_TO_TICKS(3000));
}

bool stackchan_voice_is_streaming(void)
{
    return s_stream_active;
}

esp_err_t stackchan_voice_speaker_selftest(void)
{
    if (!s_started) {
        stackchan_voice_start();
    }
    /* 0.4 s of 440 Hz sine at 16 kHz mono. */
    const size_t n = VOICE_SAMPLE_RATE * 4 / 10;
    int16_t *tone = (int16_t *)heap_caps_malloc(n * sizeof(int16_t), MALLOC_CAP_8BIT);
    if (tone == NULL) {
        return ESP_ERR_NO_MEM;
    }
    for (size_t i = 0; i < n; i++) {
        tone[i] = (int16_t)(8000.0 * sin(2.0 * M_PI * 440.0 * (double)i / VOICE_SAMPLE_RATE));
    }
    stackchan_voice_set_speaking(true);
    ESP_LOGI(TAG, "speaker self-test: playing 440 Hz PCM tone (%u samples)", (unsigned)n);
    esp_err_t err = voice_post(CMD_SELFTEST, tone, n, VOICE_SAMPLE_RATE, 0, false,
                               pdMS_TO_TICKS(5000));
    free(tone); /* owner played it synchronously (playRaw copies for one-shot) */
    ESP_LOGI(TAG, "speaker self-test done (err=%s)", esp_err_to_name(err));
    return err;
}

/* ------------------------------------------------------------------ */
/* Serial REPL command: drive the loop without a physical JoyC button. */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* E2E autonomous conversation test: inject a canned utterance through  */
/* the EXACT mic_chunk_cb uplink path (no physical mic) so it flows to   */
/* the REAL gateway as appendAudio -> transcript -> reply -> TTS.        */
/* ------------------------------------------------------------------ */

/* Minimal WAV loader: scans RIFF chunks for `data`, returns int16 PCM in a
 * freshly heap_caps_malloc'd (PSRAM-preferred) buffer. Assumes 16 kHz mono
 * s16le (the format we pack utterance.wav as). Caller frees. */
#define E2E_MAX_SAMPLES (6 * 16000) /* cap 6 s */
static int16_t *e2e_load_wav(const char *path, size_t *out_samples)
{
    *out_samples = 0;
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "E2E: cannot open %s", path);
        return NULL;
    }
    char riff[12];
    if (fread(riff, 1, 12, f) != 12 ||
        memcmp(riff, "RIFF", 4) != 0 || memcmp(riff + 8, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "E2E: %s is not a RIFF/WAVE file", path);
        fclose(f);
        return NULL;
    }
    uint32_t data_len = 0;
    for (;;) {
        uint8_t ck[8];
        if (fread(ck, 1, 8, f) != 8) { break; }
        uint32_t sz = (uint32_t)ck[4] | ((uint32_t)ck[5] << 8) |
                      ((uint32_t)ck[6] << 16) | ((uint32_t)ck[7] << 24);
        if (memcmp(ck, "data", 4) == 0) { data_len = sz; break; }
        fseek(f, (long)(sz + (sz & 1)), SEEK_CUR); /* chunks are word-aligned */
    }
    if (data_len == 0) {
        ESP_LOGE(TAG, "E2E: no data chunk in %s", path);
        fclose(f);
        return NULL;
    }
    size_t samples = data_len / sizeof(int16_t);
    if (samples > E2E_MAX_SAMPLES) { samples = E2E_MAX_SAMPLES; }
    int16_t *pcm = (int16_t *)heap_caps_malloc(samples * sizeof(int16_t),
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (pcm == NULL) {
        pcm = (int16_t *)heap_caps_malloc(samples * sizeof(int16_t), MALLOC_CAP_8BIT);
    }
    if (pcm == NULL) {
        ESP_LOGE(TAG, "E2E: PCM alloc failed (%u samples)", (unsigned)samples);
        fclose(f);
        return NULL;
    }
    size_t got = fread(pcm, sizeof(int16_t), samples, f);
    fclose(f);
    *out_samples = got;
    return pcm;
}

/* Drive ONE full conversation turn autonomously. Emits stable scrapeable
 * tokens (E2E: / STATE -> ...) for the python harness. Runs on the console
 * task; gateway RPC callbacks + owner playback run on their own tasks. */
static int e2e_run(const char *path)
{
    printf("E2E: start\n");
    if (!s_started) {
        stackchan_voice_start();
    }
    if (!stackchan_voice_net_talk_enabled()) {
        printf("E2E: FAIL talk_mode_disabled (run `voice talk-enable` + reboot)\n");
        return 1;
    }
    if (s_mic_cb == NULL || s_ptt_cb == NULL) {
        printf("E2E: FAIL net_transport_not_registered\n");
        return 1;
    }
    size_t nsamples = 0;
    int16_t *pcm = e2e_load_wav(path, &nsamples);
    if (pcm == NULL || nsamples == 0) {
        printf("E2E: FAIL load_utterance %s\n", path);
        if (pcm) { free(pcm); }
        return 1;
    }
    printf("E2E: loaded %u samples (%.2f s) from %s\n",
           (unsigned)nsamples, (double)nsamples / (double)VOICE_SAMPLE_RATE, path);

    /* --- LISTENING / uplink phase: open the gateway turn, no physical mic --- */
    s_listen_intent = true;
    s_await_reply = false;
    printf("STATE -> LISTENING\n");
    s_ptt_cb(true, s_ptt_cb_ctx);          /* net.c: talk.session.create */
    vTaskDelay(pdMS_TO_TICKS(800));        /* let the session open before we stream */

    /* Stream the canned PCM through the SAME sink the physical mic uses, paced
     * in real time (100 ms / 1600-sample chunks) so net.c appendAudio keeps up
     * exactly as with a live utterance. */
    uint32_t chunks = 0;
    for (size_t off = 0; off < nsamples; off += REC_CHUNK_SAMPLES) {
        size_t n = (nsamples - off < REC_CHUNK_SAMPLES) ? (nsamples - off)
                                                        : REC_CHUNK_SAMPLES;
        s_mic_cb(pcm + off, n, s_mic_cb_ctx);
        chunks++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("E2E: injected %u samples in %u chunks\n", (unsigned)nsamples, (unsigned)chunks);
    /* a little tail so the last appendAudio lands before we close the turn */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* --- PTT-off: ARM the confirm gate EXACTLY like the real path, then close
     * the turn. The pink loading LED must NOT light here; it lights only when
     * the gateway returns a transcript (turn_flush -> on_gateway_activity). --- */
    s_listen_intent = false;
    s_await_reply = true;
    s_await_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(AWAIT_REPLY_TIMEOUT_MS);
    voice_post(CMD_E2E_ARM, NULL, 0, 0, 0, false, pdMS_TO_TICKS(2000)); /* state THINKING, no LED */
    printf("E2E: armed (awaiting gateway confirm)\n");
    s_ptt_cb(false, s_ptt_cb_ctx);         /* net.c: talk.session.close -> STT final -> reply */

    free(pcm);
    printf("E2E: uplink complete; awaiting transcript + reply (async)\n");
    return 0;
}

static int handle_voice_command(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "e2e") == 0) {
        const char *path = (argc >= 3) ? argv[2] : "/spiffs/utterance.wav";
        return e2e_run(path);
    }
    if (argc == 2 && strcmp(argv[1], "on") == 0) {
        if (!s_listen_intent) {
            stackchan_voice_toggle_listening();
        }
        printf("voice listening=1\n");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "off") == 0) {
        if (s_listen_intent) {
            stackchan_voice_toggle_listening();
        }
        printf("voice listening=0\n");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "toggle") == 0) {
        bool now = stackchan_voice_toggle_listening();
        printf("voice listening=%d\n", (int)now);
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "test") == 0) {
        esp_err_t err = stackchan_voice_speaker_selftest();
        printf("voice speaker self-test: %s\n", esp_err_to_name(err));
        return err == ESP_OK ? 0 : 1;
    }
    if (argc >= 2 && strcmp(argv[1], "streamtest") == 0) {
        /* Push a continuous 440 Hz tone through the SAME stream_begin/feed/end
         * path used by speaker.playStream, in 128 ms chunks, no network in the
         * loop. Phase carried across chunks so any gap/click is a playback seam.
         * Optional arg: sample rate (default 16000) e.g. `voice streamtest 24000`. */
        const uint32_t rate = (argc >= 3) ? (uint32_t)atoi(argv[2]) : 16000;
        const size_t chunk = 2048;            /* 128 ms @ 16 kHz */
        const int chunks = 40;                /* ~5 s total */
        int16_t *buf = (int16_t *)heap_caps_malloc(chunk * sizeof(int16_t), MALLOC_CAP_8BIT);
        if (buf == NULL) { printf("streamtest: OOM\n"); return 1; }
        esp_err_t err = stackchan_voice_stream_begin(rate, 160);
        if (err != ESP_OK) { free(buf); printf("streamtest begin: %s\n", esp_err_to_name(err)); return 1; }
        double ph = 0.0;
        const double dp = 2.0 * M_PI * 440.0 / (double)rate;
        for (int c = 0; c < chunks; c++) {
            for (size_t i = 0; i < chunk; i++) {
                buf[i] = (int16_t)(8000.0 * sin(ph));
                ph += dp;
                if (ph > 2.0 * M_PI) ph -= 2.0 * M_PI;
            }
            stackchan_voice_stream_feed(buf, chunk);
        }
        stackchan_voice_stream_end();
        free(buf);
        printf("streamtest done (continuous 440Hz tone; clicks == seams)\n");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "status") == 0) {
        printf("voice: started=%d listening=%d state=%s streaming=%d talk_enabled=%d\n",
               (int)s_started, (int)s_listen_intent, state_name(s_state),
               (int)s_stream_active, (int)stackchan_voice_net_talk_enabled());
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "talk-enable") == 0) {
        esp_err_t err = stackchan_voice_net_set_talk_enabled(true);
        printf("talk mode enabled=%s. REBOOT, then approve the node re-pair "
               "(openclaw nodes pending/approve or openclaw devices approve).\n",
               err == ESP_OK ? "ok" : esp_err_to_name(err));
        return err == ESP_OK ? 0 : 1;
    }
    if (argc == 2 && strcmp(argv[1], "talk-disable") == 0) {
        esp_err_t err = stackchan_voice_net_set_talk_enabled(false);
        printf("talk mode disabled=%s. REBOOT to restore the baseline scope surface.\n",
               err == ESP_OK ? "ok" : esp_err_to_name(err));
        return err == ESP_OK ? 0 : 1;
    }
    if (argc == 2 && strcmp(argv[1], "talk-status") == 0) {
        printf("talk mode enabled=%d\n", (int)stackchan_voice_net_talk_enabled());
        return 0;
    }
    printf("usage: voice on | off | toggle | test | status | streamtest [rate] | "
           "e2e [wavpath] | talk-enable | talk-disable | talk-status\n");
    return 1;
}

esp_err_t stackchan_voice_register_command(void)
{
    esp_console_cmd_t cmd = {};
    cmd.command = "voice";
    cmd.help = "Push-to-talk voice loop: mic capture + speaker self-test.";
    cmd.hint = "on|off|toggle|test|status|streamtest|e2e|talk-enable|talk-disable|talk-status";
    cmd.func = handle_voice_command;
    return esp_console_cmd_register(&cmd);
}
