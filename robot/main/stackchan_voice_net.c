/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * StackChan voice-loop network transport. See stackchan_voice_net.h.
 */

#include "stackchan_voice_net.h"
#include "stackchan_voice.h"
#include "stackchan_body.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_console.h"
#include "nvs.h"
#include "mbedtls/base64.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

static const char *TAG = "voice_net";

/* DEBUG streamed-audio capture (defined lower; declared here for early callers) */
static void micdump_reset(void);
static void micdump_emit(void);

/* Global mute flag read by the WireGuard hot-path printf (wireguardif.c) so the
 * per-packet [WG_RX] flood can't shred the base64 audio dump over USB-CDC. */
volatile bool g_micdump_quiet = false;

/* Session key the transcript is injected into and replies are watched on.
 * Fully-qualified agent:<id>:<mainKey> so this device is HARD-BOUND to the
 * "stackchan" agent regardless of the gateway's default agent. The gateway
 * only re-scopes bare alias keys (e.g. "main") to the default agent; a
 * fully-qualified "agent:..." key is routed verbatim to that exact agent
 * (classifySessionKeyShape -> "agent" -> resolveAgentIdFromSessionKey). The
 * "stackchan" agent must exist on the paired gateway. */
#define VOICE_SESSION_KEY "agent:stackchan:main"
/* Reply broadcast comes back under the same canonical store key the transcript
 * was injected into, so the subscribe key MUST equal VOICE_SESSION_KEY exactly
 * or replies are silently dropped (subscriber registry keyed by exact string). */
#define CHAT_SUBSCRIBE_KEY "agent:stackchan:main"

/* Gateway transcription relay input contract: WIDEBAND PCM 16 kHz mono
 * (ElevenLabs scribe_v2_realtime audio_format=pcm_16000). Mic already captures
 * PCM 16 kHz, so we send it through UNTOUCHED except a DC block + mild gain --
 * NO downsample, NO μ-law. Full 16 kHz keeps the consonant energy (s/f/th/sh/t)
 * that 8 kHz μ-law telephony was throwing away, which is the transcript-quality
 * win. Requires gateway config audioFormat=pcm_16000, sampleRate=16000. */
#define RELAY_IN_RATE_HZ 16000

/* Bound the outbound audio queue so a slow tunnel applies backpressure by
 * dropping oldest frames rather than exhausting the heap. Each item is one
 * μ-law encoded ~100 ms chunk (~800 bytes). */
#define AUDIO_Q_DEPTH 16

typedef struct {
    uint8_t *data;   /* signed 16-bit LE PCM, 16 kHz mono */
    size_t   len;    /* bytes */
} audio_frame_t;

/* ---- Transport state ---- */
static esp_openclaw_node_handle_t s_node = NULL;
/* Node-role leg. talk.* RPCs ride the operator leg (s_node), but node.event
 * (voice.transcript / chat.subscribe) is a NODE-ROLE-ONLY gateway method: the
 * gateway's role-policy rejects it from an operator-role connection
 * ("unauthorized role: operator"). So those two calls MUST go over the node
 * leg. Set via stackchan_voice_net_set_node_leg() from app_main once the
 * node-role client exists. Falls back to s_node when talk runs single-leg
 * (no talk mode) where s_node is itself the node-role client. */
static esp_openclaw_node_handle_t s_node_ctrl = NULL;

static esp_openclaw_node_handle_t node_event_leg(void)
{
    return s_node_ctrl != NULL ? s_node_ctrl : s_node;
}

void stackchan_voice_net_set_node_leg(esp_openclaw_node_handle_t node)
{
    s_node_ctrl = node;
}
static QueueHandle_t s_audio_q = NULL;
static SemaphoreHandle_t s_state_lock = NULL;

static char s_session_id[128] = {0};   /* active transcription sessionId */
static volatile bool s_session_opening = false;
static volatile bool s_listening = false;
static volatile bool s_subscribed = false;

/* De-dupe replies: remember the last transcript we injected + last reply spoken. */
static char s_last_reply[256] = {0};

/* Turn accumulation: the transcription relay emits a `transcript.done` on every
 * VAD silence gap, so a mid-sentence pause used to fire a response. Instead we
 * BUFFER every final for the whole PTT hold and inject ONCE, only after the mic
 * is released (PTT off). A short debounce after close lets the last flushed
 * final land before we submit. Mic-end is now the sole response trigger. */
#define TURN_BUF_BYTES 2048
#define TURN_FLUSH_DEBOUNCE_MS 900
static char s_turn_buf[TURN_BUF_BYTES] = {0};
static size_t s_turn_len = 0;
static volatile bool s_turn_closing = false;   /* PTT released; awaiting final */
static TimerHandle_t s_flush_timer = NULL;

static void inject_transcript(const char *text);

/* Append one STT final to the turn buffer (space-joined). */
static void turn_buf_append(const char *text)
{
    if (text == NULL || text[0] == '\0') {
        return;
    }
    if (s_turn_len > 0 && s_turn_len < TURN_BUF_BYTES - 1) {
        s_turn_buf[s_turn_len++] = ' ';
        s_turn_buf[s_turn_len] = '\0';
    }
    size_t avail = TURN_BUF_BYTES - 1 - s_turn_len;
    if (avail == 0) {
        return;
    }
    strlcpy(s_turn_buf + s_turn_len, text, avail + 1);
    s_turn_len += strnlen(s_turn_buf + s_turn_len, avail);
}

/* Inject the whole accumulated turn as ONE message, then reset. */
static void turn_flush(void)
{
    s_turn_closing = false;
    if (s_turn_len == 0) {
        return;
    }
    ESP_LOGI(TAG, "turn flush -> inject full turn: \"%.80s\"", s_turn_buf);
    inject_transcript(s_turn_buf);
    /* Confirm-gated loading state: we only reach here with a non-empty turn,
     * which means the gateway received our audio and STT returned a transcript
     * (proof the turn landed) and we are now asking the agent to respond. This
     * is the earliest trustworthy "received + responding" signal that also
     * fires AFTER PTT-off/mic teardown, so lighting the loading LED here is
     * safe (no codec/LED I2C clash) and correct. If the gateway is unreachable,
     * no STT final ever arrives, s_turn_len stays 0, this function returns early
     * above, the hook is never called, and the loading LED never lights. This
     * is a display hook only -- it does not alter any STT/TTS logic. */
    stackchan_voice_on_gateway_activity();
    s_turn_buf[0] = '\0';
    s_turn_len = 0;
    micdump_emit(); /* DEBUG: dump the streamed audio for this utterance */
}

static void flush_timer_cb(TimerHandle_t t)
{
    (void)t;
    turn_flush();
}

/* Talk mode is opt-in: advertising operator.read/operator.write scopes forces a
 * one-time node re-approval (the gateway rejects the old device token with
 * AUTH_SCOPE_MISMATCH until then). Default OFF so the baseline connection keeps
 * working out of the box; Eric flips it on with `voice talk-enable` + reboot,
 * then approves the pending re-pair. */
#define VOICE_NVS_NS   "stackchan_voice"
#define VOICE_NVS_KEY  "talk_enabled"

static bool talk_mode_enabled(void)
{
    nvs_handle_t h;
    if (nvs_open(VOICE_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    uint8_t v = 0;
    esp_err_t err = nvs_get_u8(h, VOICE_NVS_KEY, &v);
    nvs_close(h);
    return err == ESP_OK && v != 0;
}

static esp_err_t talk_mode_set(bool enabled)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(VOICE_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u8(h, VOICE_NVS_KEY, enabled ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

/* ------------------------------------------------------------------ */
/* 16 kHz passthrough: raw PCM only, ZERO device-side DSP               */
/* ------------------------------------------------------------------ */

/* Emit native 16 kHz mic PCM as signed 16-bit LITTLE-ENDIAN bytes for
 * talk.session.appendAudio (ElevenLabs pcm_16000). Deliberately NO gain, NO
 * DC block, NO low-pass, NO decimation. ElevenLabs runs its own AGC/denoise,
 * so any on-device conditioning just fights the provider and clips peaks. Mic
 * magnification is already tamed at capture (see stackchan_voice.cpp). This is
 * a pure byte repack; out must hold samples*2 bytes. Returns byte count. */
static size_t pcm16k_to_pcm16k(const int16_t *pcm, size_t samples, uint8_t *out)
{
    for (size_t i = 0; i < samples; i++) {
        int16_t s = pcm[i];
        out[2 * i]     = (uint8_t)(s & 0xFF);          /* little-endian lo */
        out[2 * i + 1] = (uint8_t)((s >> 8) & 0xFF);   /* little-endian hi */
    }
    return samples * 2; /* bytes */
}

/* ------------------------------------------------------------------ */
/* RPC result callbacks (run on the node component task; keep short)   */
/* ------------------------------------------------------------------ */

static void on_append_result(
    esp_openclaw_node_handle_t node,
    const esp_openclaw_node_gateway_result_t *result,
    void *user_ctx)
{
    (void)node;
    (void)user_ctx;
    if (!result->ok) {
        ESP_LOGW(TAG, "appendAudio failed: %s %s",
                 result->error_code ? result->error_code : "?",
                 result->error_message ? result->error_message : "");
    } else {
        /* Telemetry only (no STT/TTS logic change): a scrapeable success token so
         * the E2E harness can assert the uplink landed. Counter proves >=1. */
        static uint32_t s_append_ok = 0;
        ESP_LOGI(TAG, "appendAudio ok (#%u)", (unsigned)(++s_append_ok));
    }
}

static void on_session_create(
    esp_openclaw_node_handle_t node,
    const esp_openclaw_node_gateway_result_t *result,
    void *user_ctx)
{
    (void)node;
    (void)user_ctx;
    s_session_opening = false;
    if (!result->ok || result->payload_json == NULL) {
        ESP_LOGE(TAG, "talk.session.create failed: %s %s",
                 result->error_code ? result->error_code : "?",
                 result->error_message ? result->error_message : "");
        return;
    }
    cJSON *payload = cJSON_Parse(result->payload_json);
    cJSON *sid = payload ? cJSON_GetObjectItemCaseSensitive(payload, "sessionId") : NULL;
    if (cJSON_IsString(sid) && sid->valuestring != NULL) {
        xSemaphoreTake(s_state_lock, portMAX_DELAY);
        strlcpy(s_session_id, sid->valuestring, sizeof(s_session_id));
        xSemaphoreGive(s_state_lock);
        ESP_LOGI(TAG, "talk transcription session open: %s", s_session_id);
    } else {
        ESP_LOGE(TAG, "talk.session.create: no sessionId in payload");
    }
    cJSON_Delete(payload);
}

static void on_speak_result(
    esp_openclaw_node_handle_t node,
    const esp_openclaw_node_gateway_result_t *result,
    void *user_ctx)
{
    (void)node;
    (void)user_ctx;
    if (!result->ok || result->payload_json == NULL) {
        ESP_LOGW(TAG, "talk.speak failed: %s %s",
                 result->error_code ? result->error_code : "?",
                 result->error_message ? result->error_message : "");
        return;
    }
    cJSON *payload = cJSON_Parse(result->payload_json);
    cJSON *audio = payload ? cJSON_GetObjectItemCaseSensitive(payload, "audioBase64") : NULL;
    cJSON *fmt = payload ? cJSON_GetObjectItemCaseSensitive(payload, "outputFormat") : NULL;
    if (!cJSON_IsString(audio) || audio->valuestring == NULL) {
        ESP_LOGW(TAG, "talk.speak: no audioBase64");
        cJSON_Delete(payload);
        return;
    }
    const char *fmt_s = cJSON_IsString(fmt) ? fmt->valuestring : "pcm_16000";
    /* Only raw pcm_* is playable directly. Derive the sample rate from the
     * format tag (e.g. pcm_16000 -> 16000). */
    uint32_t rate = 16000;
    const char *us = strrchr(fmt_s, '_');
    if (us != NULL) {
        long r = strtol(us + 1, NULL, 10);
        if (r >= 8000 && r <= 48000) {
            rate = (uint32_t)r;
        }
    }
    if (strncmp(fmt_s, "pcm", 3) != 0) {
        ESP_LOGW(TAG, "talk.speak returned non-PCM format '%s'; set talk pcm_16000. Skipping playback.", fmt_s);
        cJSON_Delete(payload);
        return;
    }

    size_t b64_len = strlen(audio->valuestring);
    size_t pcm_cap = (b64_len / 4) * 3 + 4;
    uint8_t *pcm = heap_caps_malloc(pcm_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (pcm == NULL) {
        pcm = heap_caps_malloc(pcm_cap, MALLOC_CAP_8BIT);
    }
    if (pcm == NULL) {
        ESP_LOGE(TAG, "talk.speak: PCM alloc failed (%u bytes)", (unsigned)pcm_cap);
        cJSON_Delete(payload);
        return;
    }
    size_t pcm_len = 0;
    int rc = mbedtls_base64_decode(pcm, pcm_cap, &pcm_len,
                                   (const uint8_t *)audio->valuestring, b64_len);
    cJSON_Delete(payload);
    if (rc != 0) {
        ESP_LOGE(TAG, "talk.speak: base64 decode failed rc=%d", rc);
        free(pcm);
        return;
    }

    ESP_LOGI(TAG, "talk.speak audio: %u PCM bytes @ %u Hz -> speaker",
             (unsigned)pcm_len, (unsigned)rate);
    stackchan_thinking_set(false); /* reply audio arriving: leave waiting state */
    stackchan_voice_set_speaking(true);
    stackchan_voice_play_pcm((const int16_t *)pcm, pcm_len / sizeof(int16_t), rate);
    stackchan_voice_set_speaking(false);
    free(pcm);
}

static void on_generic_result(
    esp_openclaw_node_handle_t node,
    const esp_openclaw_node_gateway_result_t *result,
    void *user_ctx)
{
    (void)node;
    const char *what = (const char *)user_ctx;
    if (!result->ok) {
        ESP_LOGW(TAG, "%s failed: %s %s", what ? what : "rpc",
                 result->error_code ? result->error_code : "?",
                 result->error_message ? result->error_message : "");
    }
}

/* chat.subscribe result: only latch s_subscribed on genuine success so a
 * pre-connect / wrong-leg attempt doesn't wedge us as "subscribed" and block
 * the lazy retry on the next PTT. */
static void on_subscribe_result(
    esp_openclaw_node_handle_t node,
    const esp_openclaw_node_gateway_result_t *result,
    void *user_ctx)
{
    (void)node;
    (void)user_ctx;
    if (result->ok) {
        s_subscribed = true;
        ESP_LOGI(TAG, "subscribed to chat replies on session '%s'", CHAT_SUBSCRIBE_KEY);
    } else {
        s_subscribed = false; /* allow retry on next PTT */
        ESP_LOGW(TAG, "chat.subscribe failed: %s %s (will retry)",
                 result->error_code ? result->error_code : "?",
                 result->error_message ? result->error_message : "");
    }
}

/* ------------------------------------------------------------------ */
/* Outbound RPC helpers                                                */
/* ------------------------------------------------------------------ */

static void open_session(void)
{
    if (s_node == NULL) {
        return;
    }
    s_session_opening = true;
    s_session_id[0] = '\0';
    const char *params =
        "{\"mode\":\"transcription\",\"transport\":\"gateway-relay\",\"brain\":\"none\"}";
    esp_err_t err = esp_openclaw_node_gateway_request(
        s_node, "talk.session.create", params, on_session_create, NULL);
    if (err != ESP_OK) {
        s_session_opening = false;
        ESP_LOGW(TAG, "talk.session.create submit failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "opening talk transcription session ...");
    }
}

static void stop_turn_and_close(void)
{
    if (s_node == NULL || s_session_id[0] == '\0') {
        return;
    }
    char params[192];
    /* cancelTurn is a no-op-safe way to commit/close the current turn on the
     * transcription relay; the relay emits transcript.done then closes. */
    snprintf(params, sizeof(params), "{\"sessionId\":\"%s\"}", s_session_id);
    esp_openclaw_node_gateway_request(
        s_node, "talk.session.close", params, on_generic_result, (void *)"talk.session.close");
}

/* ------------------------------------------------------------------ */
/* DEBUG: capture the EXACT bytes we ship to talk.session.appendAudio   */
/* so Eric can pull the streamed audio off serial and analyze it. This  */
/* taps AFTER the queue (so any backpressure drops are already applied) */
/* -- it is byte-for-byte what the gateway receives. Accumulate a whole */
/* utterance in PSRAM, then base64-dump between markers on turn flush.  */
/* Debug streamed-audio capture. OFF in prod: the base64 serial dump stalls turn
 * injection and burns PSRAM. Flip to true only for bench debugging over USB. */
#define MICDUMP_ENABLED 0
#define MICDUMP_CAP_BYTES (6 * 16000 * 2) /* 6 s @ 16k mono s16le */
static uint8_t *s_micdump = NULL;
static size_t   s_micdump_len = 0;
static bool     s_micdump_trunc = false;

static void micdump_reset(void)
{
    s_micdump_len = 0;
    (void)s_micdump_trunc;
    s_micdump_trunc = false;
}

static void micdump_append(const uint8_t *data, size_t len)
{
    if (!MICDUMP_ENABLED) {
        return;
    }
    if (s_micdump == NULL) {
        s_micdump = heap_caps_malloc(MICDUMP_CAP_BYTES, MALLOC_CAP_SPIRAM);
        if (s_micdump == NULL) {
            return; /* no PSRAM -> capture disabled, streaming unaffected */
        }
    }
    if (s_micdump_len + len > MICDUMP_CAP_BYTES) {
        len = (s_micdump_len < MICDUMP_CAP_BYTES) ? (MICDUMP_CAP_BYTES - s_micdump_len) : 0;
        s_micdump_trunc = true;
    }
    if (len > 0) {
        memcpy(s_micdump + s_micdump_len, data, len);
        s_micdump_len += len;
    }
}

/* Base64-dump the accumulated utterance over the console between clear markers.
 * Printed AFTER the turn is done, so there's no real-time UART pressure on the
 * stream itself. Rebuild on the Mac: extract base64 between BEGIN/END, decode,
 * wrap as a 16 kHz mono s16le WAV. */
static void micdump_emit(void)
{
    if (!MICDUMP_ENABLED || s_micdump == NULL || s_micdump_len == 0) {
        return;
    }
    size_t b64_cap = ((s_micdump_len + 2) / 3) * 4 + 1;
    char *b64 = heap_caps_malloc(b64_cap, MALLOC_CAP_SPIRAM);
    if (b64 == NULL) {
        ESP_LOGW(TAG, "micdump: b64 alloc failed (%u bytes)", (unsigned)b64_cap);
        return;
    }
    size_t b64_len = 0;
    if (mbedtls_base64_encode((uint8_t *)b64, b64_cap, &b64_len, s_micdump, s_micdump_len) != 0) {
        free(b64);
        return;
    }
    /* CRC32 over the RAW PCM so the host can PROVE the dump arrived intact.
     * If host-recomputed CRC matches, any garble is REAL mic capture; if it
     * mismatches, it's serial-transport loss (my tool), not the device. */
    uint32_t crc = esp_rom_crc32_le(0, s_micdump, s_micdump_len);

    /* Mute BOTH the ESP_LOG flood AND the raw WireGuard per-packet printf flood
     * ([WG_RX] etc.) while we stream the base64, or they steal the USB-CDC TX
     * FIFO and corrupt the dump. Restore after. */
    g_micdump_quiet = true;
    esp_log_level_set("*", ESP_LOG_NONE);
    printf("\n---MICDUMP BEGIN bytes=%u sr=16000 ch=1 bits=16 trunc=%d crc32=%08x---\n",
           (unsigned)s_micdump_len, s_micdump_trunc ? 1 : 0, (unsigned)crc);
    /* Small paced blocks: the ESP32-S3 USB-CDC TX ring silently DROPS on
     * overflow, so send modest lines and yield between them to let the host
     * drain. Slower but lossless -- correctness over speed for a debug dump. */
    const size_t LINE = 256;
    for (size_t off = 0; off < b64_len; off += LINE) {
        size_t n = (b64_len - off < LINE) ? (b64_len - off) : LINE;
        fwrite(b64 + off, 1, n, stdout);
        fputc('\n', stdout);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(8)); /* pace so the CDC FIFO never overruns */
    }
    printf("---MICDUMP END---\n");
    fflush(stdout);
    esp_log_level_set("*", ESP_LOG_INFO);
    g_micdump_quiet = false;
    free(b64);
}

static void append_audio(const uint8_t *data, size_t len)
{
    if (s_node == NULL || s_session_id[0] == '\0') {
        return;
    }
    micdump_append(data, len); /* DEBUG: exact shipped bytes */
    size_t b64_cap = ((len + 2) / 3) * 4 + 1;
    char *b64 = malloc(b64_cap);
    if (b64 == NULL) {
        return;
    }
    size_t b64_len = 0;
    if (mbedtls_base64_encode((uint8_t *)b64, b64_cap, &b64_len, data, len) != 0) {
        free(b64);
        return;
    }
    /* Build params JSON. */
    size_t params_cap = b64_len + 128;
    char *params = malloc(params_cap);
    if (params == NULL) {
        free(b64);
        return;
    }
    snprintf(params, params_cap,
             "{\"sessionId\":\"%s\",\"audioBase64\":\"%s\"}", s_session_id, b64);
    esp_openclaw_node_gateway_request(
        s_node, "talk.session.appendAudio", params, on_append_result, NULL);
    free(params);
    free(b64);
}

static void speak_text(const char *text)
{
    if (s_node == NULL || text == NULL || text[0] == '\0') {
        return;
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "text", text);
    /* REQUIRED: pin the return format to raw PCM. talk.speak defaults to
     * mp3_44100_128 server-side, which on_speak_result rejects ("non-PCM"),
     * so with no outputFormat the synthesized audio is silently dropped and
     * nothing reaches the speaker. pcm_16000 matches the device DAC path. */
    cJSON_AddStringToObject(root, "outputFormat", "pcm_16000");
    char *params = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (params == NULL) {
        return;
    }
    ESP_LOGI(TAG, "talk.speak: \"%.60s\"", text);
    esp_openclaw_node_gateway_request(
        s_node, "talk.speak", params, on_speak_result, NULL);
    free(params);
}

static void subscribe_chat(void);

static void inject_transcript(const char *text)
{
    esp_openclaw_node_handle_t leg = node_event_leg();
    if (leg == NULL || text == NULL || text[0] == '\0') {
        return;
    }
    /* Lazily (re)subscribe to chat replies now that the node leg is up and
     * connected. Guarded by s_subscribed; no-op once it sticks. */
    subscribe_chat();
    /* node.event voice.transcript -> injected as a normal user message on the
     * VOICE_SESSION_KEY session. */
    cJSON *payload = cJSON_CreateObject();
    cJSON_AddStringToObject(payload, "text", text);
    cJSON_AddStringToObject(payload, "sessionKey", VOICE_SESSION_KEY);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "event", "voice.transcript");
    cJSON_AddItemToObject(root, "payload", payload);
    char *params = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (params == NULL) {
        return;
    }
    ESP_LOGI(TAG, "inject transcript -> agent: \"%.60s\"", text);
    esp_openclaw_node_gateway_request(
        leg, "node.event", params, on_generic_result, (void *)"node.event(voice.transcript)");
    free(params);
}

static void subscribe_chat(void)
{
    esp_openclaw_node_handle_t leg = node_event_leg();
    /* Require the NODE leg specifically: chat.subscribe is node-role-only, and
     * the reply broadcast comes back on whichever leg subscribed, so it must be
     * the node leg (whose gateway_event_cb is wired into this transport). Skip
     * until the node leg exists so we don't burn the attempt on the operator
     * leg during early boot. */
    if (s_node_ctrl == NULL || leg == NULL || s_subscribed) {
        return;
    }
    cJSON *payload = cJSON_CreateObject();
    cJSON_AddStringToObject(payload, "sessionKey", CHAT_SUBSCRIBE_KEY);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "event", "chat.subscribe");
    cJSON_AddItemToObject(root, "payload", payload);
    char *params = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (params == NULL) {
        return;
    }
    esp_openclaw_node_gateway_request(
        leg, "node.event", params, on_subscribe_result, (void *)"node.event(chat.subscribe)");
    free(params);
}

/* ------------------------------------------------------------------ */
/* Image ingest: hand a JPEG to OpenClaw as an agent.request with an    */
/* image attachment (node-role node.event).                            */
/* ------------------------------------------------------------------ */
/* The gateway's handleNodeEvent(agent.request) parses payload.attachments
 * (normalizeRpcAttachmentsToChatAttachments -> parseMessageWithAttachments):
 * each attachment is { type:"image", mimeType, fileName, content:<base64> }
 * (a data-URL prefix would be stripped; raw base64 is accepted). The image is
 * injected as a user message on `sessionKey`, which we point at the SAME voice
 * session so the model's reply returns via the existing chat-subscribe path.
 * agent.request is node-role-only, so it MUST go over the node leg. */
esp_err_t stackchan_voice_net_send_image(
    const char *message, const uint8_t *jpeg, size_t jpeg_len)
{
    esp_openclaw_node_handle_t leg = node_event_leg();
    if (leg == NULL) {
        ESP_LOGW(TAG, "send_image: no node leg (not connected yet)");
        return ESP_ERR_INVALID_STATE;
    }
    if (jpeg == NULL || jpeg_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (message == NULL) {
        message = "Describe what you see in this photo.";
    }

    /* Base64 the JPEG into PSRAM (can be tens of KB). */
    size_t b64_cap = ((jpeg_len + 2) / 3) * 4 + 1;
    char *b64 = heap_caps_malloc(b64_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (b64 == NULL) {
        b64 = malloc(b64_cap);
    }
    if (b64 == NULL) {
        return ESP_ERR_NO_MEM;
    }
    size_t b64_len = 0;
    if (mbedtls_base64_encode((uint8_t *)b64, b64_cap, &b64_len, jpeg, jpeg_len) != 0) {
        free(b64);
        return ESP_FAIL;
    }

    /* Build the node.event params by hand (a cJSON tree would copy the whole
     * base64 blob onto the internal heap during printing; a direct PSRAM
     * assembly keeps peak internal-heap use low while TLS is active). `message`
     * is a plain ASCII prompt with no JSON-special characters, so it is emitted
     * verbatim. */
    static const char *PFX =
        "{\"event\":\"agent.request\",\"payload\":{\"sessionKey\":\""
        VOICE_SESSION_KEY
        "\",\"message\":\"";
    static const char *MID =
        "\",\"attachments\":[{\"type\":\"image\",\"mimeType\":\"image/jpeg\","
        "\"fileName\":\"capture.jpg\",\"content\":\"";
    static const char *SFX = "\"}]}}";

    size_t msg_len = strlen(message);
    size_t params_cap = strlen(PFX) + msg_len + strlen(MID) + b64_len + strlen(SFX) + 1;
    char *params = heap_caps_malloc(params_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (params == NULL) {
        params = malloc(params_cap);
    }
    if (params == NULL) {
        free(b64);
        return ESP_ERR_NO_MEM;
    }
    char *w = params;
    w = stpcpy(w, PFX);
    w = stpcpy(w, message);
    w = stpcpy(w, MID);
    memcpy(w, b64, b64_len); w += b64_len; *w = '\0';
    w = stpcpy(w, SFX);
    free(b64);

    ESP_LOGI(TAG, "send_image: %u JPEG bytes -> agent.request (%u b64) session '%s'",
             (unsigned)jpeg_len, (unsigned)b64_len, VOICE_SESSION_KEY);
    /* Make sure we are subscribed so the analysis reply comes back to us. */
    subscribe_chat();
    esp_err_t err = esp_openclaw_node_gateway_request(
        leg, "node.event", params, on_generic_result, (void *)"node.event(agent.request/image)");
    free(params);
    return err;
}

/* ------------------------------------------------------------------ */
/* stackchan_voice sinks                                               */
/* ------------------------------------------------------------------ */

static void on_mic_chunk(const int16_t *pcm, size_t samples, void *ctx)
{
    (void)ctx;
    if (!s_listening || s_audio_q == NULL) {
        return;
    }
    size_t out_cap = samples * 2; /* 16-bit LE PCM, no decimation */
    uint8_t *data = malloc(out_cap);
    if (data == NULL) {
        return;
    }
    size_t n = pcm16k_to_pcm16k(pcm, samples, data);
    audio_frame_t frame = { .data = data, .len = n };
    if (xQueueSend(s_audio_q, &frame, 0) != pdTRUE) {
        /* Queue full: drop the OLDEST to keep latency bounded. */
        audio_frame_t old;
        if (xQueueReceive(s_audio_q, &old, 0) == pdTRUE) {
            free(old.data);
        }
        if (xQueueSend(s_audio_q, &frame, 0) != pdTRUE) {
            free(data);
        }
    }
}

static void on_ptt(bool listening, void *ctx)
{
    (void)ctx;
    s_listening = listening;
    if (listening) {
        ESP_LOGI(TAG, "PTT on -> open session + stream audio up");
        micdump_reset(); /* DEBUG: start a fresh streamed-audio capture */
        open_session();
    } else {
        ESP_LOGI(TAG, "PTT off -> close turn; will inject full turn on final");
        /* Mark the turn as closing and let the audio task drain + relay finalize.
         * The tail transcript.done arrives shortly after close; the debounce
         * timer flushes the whole accumulated turn as ONE injection. If no
         * further final arrives, the timer still fires and flushes what we have. */
        s_turn_closing = true;
        stop_turn_and_close();
        if (s_flush_timer != NULL) {
            xTimerStop(s_flush_timer, 0);
            xTimerChangePeriod(s_flush_timer, pdMS_TO_TICKS(TURN_FLUSH_DEBOUNCE_MS), 0);
            xTimerStart(s_flush_timer, 0);
        }
    }
}

/* Drains the outbound audio queue -> appendAudio. Separate task so encoding and
 * the RPC submit never block the mic capture task. */
static void audio_pump_task(void *arg)
{
    (void)arg;
    for (;;) {
        audio_frame_t frame;
        if (xQueueReceive(s_audio_q, &frame, pdMS_TO_TICKS(200)) == pdTRUE) {
            /* Only forward once a session id exists; otherwise drop (the first
             * few chunks during session open are acceptable to lose on PTT). */
            if (s_session_id[0] != '\0') {
                append_audio(frame.data, frame.len);
            }
            free(frame.data);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Gateway event ingestion (talk.event transcripts + chat replies)     */
/* ------------------------------------------------------------------ */

/* Extract the deepest transcript text from a talk.event payload. Shape:
 *   { nodeId, ..., talkEvent: { type:"transcript.done", payload:{text}, final } }
 * or transcription relay: { type:"transcript", text, final } */
static void handle_talk_event(cJSON *payload)
{
    if (!cJSON_IsObject(payload)) {
        return;
    }
    cJSON *talkEvent = cJSON_GetObjectItemCaseSensitive(payload, "talkEvent");
    cJSON *type = NULL;
    cJSON *text = NULL;
    bool final = false;
    if (cJSON_IsObject(talkEvent)) {
        type = cJSON_GetObjectItemCaseSensitive(talkEvent, "type");
        cJSON *p = cJSON_GetObjectItemCaseSensitive(talkEvent, "payload");
        if (cJSON_IsObject(p)) {
            text = cJSON_GetObjectItemCaseSensitive(p, "text");
        }
        cJSON *f = cJSON_GetObjectItemCaseSensitive(talkEvent, "final");
        final = cJSON_IsTrue(f);
    }
    /* Fallback: flat transcription-relay event ({type:"transcript",text,final}) */
    if (text == NULL) {
        text = cJSON_GetObjectItemCaseSensitive(payload, "text");
        cJSON *f = cJSON_GetObjectItemCaseSensitive(payload, "final");
        final = final || cJSON_IsTrue(f);
        type = type ? type : cJSON_GetObjectItemCaseSensitive(payload, "type");
    }
    const char *type_s = cJSON_IsString(type) ? type->valuestring : "";
    bool is_done = strcmp(type_s, "transcript.done") == 0 ||
                   (strcmp(type_s, "transcript") == 0 && final);
    if (is_done && cJSON_IsString(text) && text->valuestring != NULL &&
        text->valuestring[0] != '\0') {
        /* Do NOT inject per-final. Accumulate every final for this PTT hold and
         * let mic-end (PTT off) trigger the single injection. This prevents a
         * mid-sentence VAD pause from firing a premature response. */
        ESP_LOGI(TAG, "STT final (buffered): \"%.80s\"", text->valuestring);
        turn_buf_append(text->valuestring);
        /* If PTT is already released, this is (likely) the last final -> restart
         * the debounce so we flush shortly after the tail final lands. */
        if (s_turn_closing && s_flush_timer != NULL) {
            xTimerStop(s_flush_timer, 0);
            xTimerChangePeriod(s_flush_timer, pdMS_TO_TICKS(TURN_FLUSH_DEBOUNCE_MS), 0);
            xTimerStart(s_flush_timer, 0);
        }
    }
}

/* chat event payload carries assistant reply text. We want the final assistant
 * message. Shape varies; look for a final/complete text field. */
static const char *extract_message_text(cJSON *message)
{
    if (cJSON_IsString(message)) {
        return message->valuestring;
    }
    if (!cJSON_IsObject(message)) {
        return NULL;
    }
    /* role gate: skip non-assistant rows (user echo, tool, etc.). */
    cJSON *role = cJSON_GetObjectItemCaseSensitive(message, "role");
    if (cJSON_IsString(role) && strcmp(role->valuestring, "assistant") != 0) {
        return NULL;
    }
    cJSON *t = cJSON_GetObjectItemCaseSensitive(message, "text");
    if (!cJSON_IsString(t)) {
        t = cJSON_GetObjectItemCaseSensitive(message, "content");
    }
    if (!cJSON_IsString(t)) {
        t = cJSON_GetObjectItemCaseSensitive(message, "displayText");
    }
    return cJSON_IsString(t) ? t->valuestring : NULL;
}

/* chat event payload carries the assistant reply. Verified gateway shape
 * (broadcastChatFinal, dist chat-pg-*.js):
 *   { runId, sessionKey, seq, state:"final", message: <projected msg object> }
 * where message is a display-projected object carrying role + text/content. */
static void handle_chat_event(cJSON *payload)
{
    if (!cJSON_IsObject(payload)) {
        return;
    }
    /* Terminal marker is state=="final" (accept legacy final/done/phase too). */
    cJSON *state = cJSON_GetObjectItemCaseSensitive(payload, "state");
    cJSON *final = cJSON_GetObjectItemCaseSensitive(payload, "final");
    cJSON *done = cJSON_GetObjectItemCaseSensitive(payload, "done");
    cJSON *phase = cJSON_GetObjectItemCaseSensitive(payload, "phase");
    bool is_final = (cJSON_IsString(state) && strcmp(state->valuestring, "final") == 0) ||
                    cJSON_IsTrue(final) || cJSON_IsTrue(done) ||
                    (cJSON_IsString(phase) && strcmp(phase->valuestring, "final") == 0);
    if (!is_final) {
        return;
    }
    /* Text lives in message (verified) with legacy top-level fallbacks. */
    cJSON *message = cJSON_GetObjectItemCaseSensitive(payload, "message");
    const char *reply = extract_message_text(message);
    if (reply == NULL) {
        cJSON *t = cJSON_GetObjectItemCaseSensitive(payload, "text");
        if (!cJSON_IsString(t)) {
            t = cJSON_GetObjectItemCaseSensitive(payload, "content");
        }
        reply = cJSON_IsString(t) ? t->valuestring : NULL;
    }
    if (reply == NULL || reply[0] == '\0') {
        return;
    }
    /* Skip silent-token replies so the speaker stays quiet on NO_REPLY. */
    if (strcmp(reply, "NO_REPLY") == 0 || strcmp(reply, "no_reply") == 0) {
        return;
    }
    /* De-dupe repeated final events for the same reply. */
    if (strncmp(reply, s_last_reply, sizeof(s_last_reply) - 1) == 0) {
        return;
    }
    strlcpy(s_last_reply, reply, sizeof(s_last_reply));
    ESP_LOGI(TAG, "assistant reply final -> talk.speak");
    speak_text(reply);
}

void stackchan_voice_net_on_gateway_event(const char *event, const char *payload_json)
{
    if (event == NULL) {
        return;
    }
    if (strcmp(event, "talk.event") != 0 && strcmp(event, "chat") != 0) {
        return;
    }
    cJSON *payload = payload_json ? cJSON_Parse(payload_json) : NULL;
    if (strcmp(event, "talk.event") == 0) {
        handle_talk_event(payload);
    } else {
        handle_chat_event(payload);
    }
    cJSON_Delete(payload);
}

/* ------------------------------------------------------------------ */
/* Registration + start                                                */
/* ------------------------------------------------------------------ */

esp_err_t stackchan_voice_net_set_talk_enabled(bool enabled)
{
    return talk_mode_set(enabled);
}

bool stackchan_voice_net_talk_enabled(void)
{
    return talk_mode_enabled();
}

esp_err_t stackchan_voice_net_register(esp_openclaw_node_handle_t node)
{
    if (node == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!talk_mode_enabled()) {
        ESP_LOGI(TAG, "talk mode DISABLED (baseline connection preserved). "
                      "Run `voice talk-enable` + reboot to advertise operator.read/write "
                      "scopes, then approve the operator upgrade in `openclaw devices list`.");
        return ESP_OK;
    }

    /* Advertise the Talk capability + PTT command surface. */
    esp_err_t err = esp_openclaw_node_register_capability(node, "talk");
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register capability 'talk' failed: %s", esp_err_to_name(err));
        return err;
    }
    /* Operator-role scopes required to drive the voice loop. VERIFIED against
     * the gateway core descriptors (dist core-descriptors-*.js): talk.* all
     * require operator.write; talk.event delivery requires operator.read
     * (READ_SCOPE). There is NO `operator.talk` scope — that was a bad guess
     * and made the token unreconcilable (AUTH_SCOPE_MISMATCH with no approvable
     * upgrade). Only these two real scopes are needed:
     *   operator.write -> talk.session.create/appendAudio/startTurn/endTurn,
     *                     talk.speak, talk.mode, AND node.event injection
     *                     (voice.transcript / chat.subscribe).
     *   operator.read  -> REQUIRED to receive the `talk.event` broadcast that
     *                     carries the STT transcript.done. Node-role clients are
     *                     blocked from talk.event (server EVENT_SCOPE_GUARDS +
     *                     NODE_ALLOWED_EVENTS); an operator.read-scoped approval
     *                     is what lets the transcript reach the device.
     * Requesting operator.* upgrades this node to the operator role too (like
     * Jem's Mac mini: roles=operator,node), which is what makes the upgrade
     * show up as an approvable request in `openclaw devices list`. */
    (void)esp_openclaw_node_register_scope(node, "operator.write");
    (void)esp_openclaw_node_register_scope(node, "operator.read");

    ESP_LOGW(TAG, "talk mode ENABLED: advertising 'talk' + operator.read/operator.write. "
                  "Gateway will require an operator role/scope approval (AUTH_SCOPE_MISMATCH until approved).");
    return ESP_OK;
}

esp_err_t stackchan_voice_net_start(esp_openclaw_node_handle_t node)
{
    s_node = node;
    if (s_state_lock == NULL) {
        s_state_lock = xSemaphoreCreateMutex();
    }
    if (s_audio_q == NULL) {
        s_audio_q = xQueueCreate(AUDIO_Q_DEPTH, sizeof(audio_frame_t));
    }
    if (s_state_lock == NULL || s_audio_q == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (s_flush_timer == NULL) {
        /* One-shot; period is set each time it's armed on PTT-off. */
        s_flush_timer = xTimerCreate("voice_flush", pdMS_TO_TICKS(TURN_FLUSH_DEBOUNCE_MS),
                                     pdFALSE, NULL, flush_timer_cb);
    }

    stackchan_voice_set_mic_chunk_cb(on_mic_chunk, NULL);
    stackchan_voice_set_ptt_cb(on_ptt, NULL);

    BaseType_t ok = xTaskCreatePinnedToCore(
        audio_pump_task, "voice_pump", 4096, NULL, 5, NULL, 0);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    /* Subscribe to chat replies once (safe to retry; guarded). Deferred until
     * a session is ready is handled by the node component returning
     * INVALID_STATE, so try now and it will be a no-op if not ready yet. */
    subscribe_chat();

    ESP_LOGI(TAG, "voice network transport started");
    return ESP_OK;
}
