/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * StackChan voice loop (push-to-talk).
 *
 * Phase 1 (this file, log-verifiable):
 *   - Owns a single "listening" state toggled by the JoyC thumb button.
 *   - Debounced click -> toggle -> serial log + LED-ring feedback
 *     (amber = idle, blue = listening, green = speaking).
 *
 * Phase 2 (local, log-verifiable): on listening=on, start M5Unified mic
 *   capture (PCM 16 kHz mono), accumulate chunks, log RMS level so we can see
 *   real audio is arriving; on listening=off, log total captured duration.
 *
 * Phase 3 (local, log-verifiable): speaker self-test plays a short PCM tone
 *   through M5.Speaker and swaps the LED ring to the "speaking" color, then
 *   reverts. Proves the playback + avatar-swap path end to end minus the
 *   network transport.
 *
 * The network transport (stream mic PCM up for server-side STT; stream
 * ElevenLabs TTS PCM back down) is documented separately; those pieces need a
 * gateway restart + node re-approval that only Eric can perform.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Start the voice subsystem (idle). Safe no-op if called twice. */
esp_err_t stackchan_voice_start(void);

/**
 * @brief Toggle the push-to-talk listening state.
 *
 * Call this on a JoyC thumb-button click. Logs the transition, updates the
 * LED-ring feedback, and starts/stops mic capture.
 *
 * @return the new listening state (true = now listening).
 */
bool stackchan_voice_toggle_listening(void);

/** @brief Current listening state. */
bool stackchan_voice_is_listening(void);

/**
 * @brief Play a short PCM self-test tone through the speaker and flash the
 *        "speaking" LED state. Used to verify the Phase 3 playback path
 *        without the network. Returns ESP_OK if playback was dispatched.
 */
esp_err_t stackchan_voice_speaker_selftest(void);

/**
 * @brief Register the `voice` console command (on/off/toggle/test/status) so
 *        the push-to-talk loop can be driven over the serial REPL when no
 *        JoyC button is attached. Call once after the REPL is started.
 */
esp_err_t stackchan_voice_register_command(void);

/* --- Playback path shared with the network transport (stackchan_voice_net) --- */

/**
 * @brief Set the "speaking" avatar/LED state. true on first TTS chunk, false
 *        when playback ends. Reverts the LED ring to idle when false.
 */
void stackchan_voice_set_speaking(bool speaking);

/**
 * @brief Play a block of signed 16-bit PCM (mono) at @p sample_rate through the
 *        speaker. Releases the mic first (half-duplex I2S). Blocks until the
 *        block is queued; returns after dispatch. Used by the TTS-down path.
 */
esp_err_t stackchan_voice_play_pcm(const int16_t *pcm, size_t samples, uint32_t sample_rate);

/**
 * @brief Streaming playback (constant memory, arbitrarily long audio).
 *
 * Use for long-form / live TTS where the whole clip can't be buffered:
 *   begin(rate) -> feed(chunk) repeatedly as bytes arrive -> end().
 *
 * begin(): releases the mic, starts the speaker, sets volume.
 * feed():  queues one PCM chunk, applying backpressure (blocks briefly while
 *          the DMA queue is full) so callers can pump an HTTP body straight
 *          through without overrunning. Safe to call with small chunks.
 * end():   waits for the queue to drain, then stops the speaker.
 *
 * Only one stream may be active at a time; the capture task must be idle.
 */
esp_err_t stackchan_voice_stream_begin(uint32_t sample_rate, uint8_t volume);
esp_err_t stackchan_voice_stream_feed(const int16_t *pcm, size_t samples);
void      stackchan_voice_stream_end(void);

/**
 * @brief Barge-in: immediately stop an in-progress streaming playback (dumps
 *        queued DAC audio and tears down the speaker). Safe no-op if nothing
 *        is playing. A subsequent stream_end() from the transport is safe.
 */
esp_err_t stackchan_voice_stream_abort(void);

/** @brief True while streaming playback is active (audio is playing). */
bool stackchan_voice_is_streaming(void);

/**
 * @brief Callback invoked once per captured mic chunk (PCM 16 kHz mono int16).
 *        Runs on the capture task; keep it short and non-blocking (copy/queue).
 */
typedef void (*stackchan_voice_mic_chunk_cb_t)(const int16_t *pcm, size_t samples, void *ctx);

/** @brief Register (or clear, with NULL) the mic-chunk sink. */
void stackchan_voice_set_mic_chunk_cb(stackchan_voice_mic_chunk_cb_t cb, void *ctx);

/**
 * @brief Callbacks fired on PTT edges so the network transport can open/close a
 *        Talk turn in lockstep with the local listening state.
 */
typedef void (*stackchan_voice_ptt_cb_t)(bool listening, void *ctx);

/** @brief Register (or clear, with NULL) the PTT edge sink. */
void stackchan_voice_set_ptt_cb(stackchan_voice_ptt_cb_t cb, void *ctx);

/**
 * @brief Confirm-gated loading-state hook.
 *
 * The network transport calls this ONCE the gateway has actually confirmed it
 * received the turn (i.e. an STT transcript came back and is being injected to
 * the agent). Only then does the pink "thinking"/loading LED light. The LED is
 * NO LONGER lit on PTT-off before any confirmation -- doing so started a ~25fps
 * I2C-hammering LED animation that wedged the shared In_I2C bus during
 * M5.Mic.end() and tripped the AXP2101 power poller into a false
 * "power key -> graceful shutdown" storm. Safe no-op if the device is not
 * currently armed/awaiting a reply, or if the arm window has already elapsed.
 */
void stackchan_voice_on_gateway_activity(void);

#ifdef __cplusplus
}
#endif
