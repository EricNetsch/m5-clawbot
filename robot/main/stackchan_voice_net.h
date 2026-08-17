/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * StackChan voice-loop NETWORK TRANSPORT.
 *
 * Wires the local push-to-talk state machine (stackchan_voice) to OpenClaw's
 * built-in gateway-owned Talk protocol over the node websocket. No custom
 * gateway code: it drives the existing talk.session.* / talk.speak RPCs and
 * consumes the existing talk.event / chat broadcast events.
 *
 * Flow:
 *   1. On PTT-on: open a transcription Talk session
 *      talk.session.create({ mode:"transcription", transport:"gateway-relay",
 *      brain:"none" }). The gateway streams STT with its configured streaming
 *      transcription provider.
 *   2. While listening: each mic PCM 16 kHz chunk -> base64 ->
 *      talk.session.appendAudio({ sessionId, audioBase64 }).
 *   3. On PTT-off: stop the turn; the gateway emits transcript.done on the
 *      talk.event channel (routed here via the node gateway_event_cb). We take
 *      the final transcript and inject it as a normal user message with the
 *      node.event `voice.transcript` (message injected into session `main`).
 *   4. OpenClaw processes it and replies. We subscribe to the `chat` node
 *      event; on the assistant's final text we call talk.speak({ text }) which
 *      synthesizes via ElevenLabs and returns PCM (pcm_16000) in the RPC
 *      result. We decode + play it through stackchan_voice_play_pcm and swap
 *      the avatar to "speaking", reverting when playback ends.
 *
 * Requires the node to advertise the `talk` capability and hold the
 * `operator.read` + `operator.write` scopes (registered before connect). That
 * capability-surface change needs a one-time node re-approval by the operator.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_openclaw_node.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register the Talk capability, scopes, and commands on the node BEFORE
 *        it connects. Call after esp_openclaw_node_create and before the first
 *        connect/reconnect. Idempotent-safe to call once.
 */
esp_err_t stackchan_voice_net_register(esp_openclaw_node_handle_t node);

/**
 * @brief Start the transport runtime: hook the stackchan_voice mic/PTT sinks
 *        and prepare session state. Call once after the node is created and
 *        stackchan_voice_start() has run.
 */
esp_err_t stackchan_voice_net_start(esp_openclaw_node_handle_t node);

/**
 * @brief Provide the NODE-ROLE leg handle. node.event (voice.transcript /
 *        chat.subscribe) is a node-role-only gateway method and is rejected on
 *        the operator leg ("unauthorized role: operator"), so those calls are
 *        routed over this handle. Call once from app_main after the node-role
 *        client is created. When talk runs single-leg, the transport falls
 *        back to the operator handle passed to _start().
 */
void stackchan_voice_net_set_node_leg(esp_openclaw_node_handle_t node);

/**
 * @brief Feed a gateway broadcast event (name + payload JSON) into the
 *        transport. Wire this from the node's gateway_event_cb. Unrelated
 *        events are ignored. Safe to call for every event.
 */
void stackchan_voice_net_on_gateway_event(
    const char *event,
    const char *payload_json);

/**
 * @brief Hand a JPEG image to OpenClaw for analysis by injecting it as a user
 *        message (with an image attachment) into the same agent session the
 *        voice loop uses. Sent as node.event `agent.request` over the NODE-role
 *        leg (agent.request carries `attachments[]`; scope `node`). The model's
 *        reply comes back through the existing chat-subscribe path (spoken +
 *        visible in the session), so no extra plumbing is needed to "see" it.
 *
 * @param message  Instruction/prompt to accompany the image (plain ASCII; must
 *                  not contain characters needing JSON escaping).
 * @param jpeg      JPEG bytes.
 * @param jpeg_len  Length of @p jpeg.
 * @return ESP_OK if the request was submitted, an error otherwise.
 */
esp_err_t stackchan_voice_net_send_image(
    const char *message, const uint8_t *jpeg, size_t jpeg_len);

/**
 * @brief Enable/disable Talk mode (persisted in NVS). Enabling makes the node
 *        advertise operator.read/write scopes on the NEXT boot, which requires a
 *        one-time operator re-approval. Returns ESP_OK on success.
 */
esp_err_t stackchan_voice_net_set_talk_enabled(bool enabled);

/** @brief Whether Talk mode is currently enabled in NVS. */
bool stackchan_voice_net_talk_enabled(void);

#ifdef __cplusplus
}
#endif
