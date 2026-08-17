/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "stackchan_body.h"

#include <string.h>

#include <M5Unified.h>
#include <mbedtls/base64.h>

#include "cJSON.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
extern "C" {
#include "esp_openclaw_node_example_json.h"
#include "stackchan_voice.h"
}

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Provided by app_main.c: which WS leg a node handle belongs to. */
extern "C" const char *stackchan_gateway_leg_name(esp_openclaw_node_handle_t node);

static const char *TAG = "stackchan_body";

/* Return the last `n` chars of a string (the unique tail of a signed TTS URL),
 * so serial can compare two invocations for the same reply without dumping the
 * whole (possibly secret) URL. */
static const char *url_tail(const char *s, size_t n)
{
    size_t len = s ? strlen(s) : 0;
    return (len > n) ? (s + (len - n)) : (s ? s : "");
}

/* ------------------------------------------------------------------ */
/* speaker.playStream single-fire guard (audio double-fire fix)        */
/* ------------------------------------------------------------------ */
/* The gateway plays a reply's TTS by invoking speaker.playStream with a URL
 * that points to that reply's synthesized audio (a unique /say-<epoch>.raw).
 * The command is registered on BOTH WS legs (operator + node) under one device
 * identity, so the SAME invocation (identical URL) can be delivered twice --
 * dual-leg fan-out, or a gateway retry -- and, with no guard, each invocation
 * starts an independent playback: the device speaks the reply back-to-back
 * (the reported bug). Dedup on the URL, which is the STABLE per-reply id that
 * is identical across both legs (NOT a session id, NOT a local counter).
 *
 * playstream_admit() does an ATOMIC compare-and-set under s_playstream_mtx: it
 * checks the incoming URL against the last-consumed URL and, only if distinct,
 * records it -- both in the SAME critical section, so two legs' independent
 * dispatch tasks can't both pass the check (no TOCTOU). The consumed URL is
 * retained until a DISTINCT reply URL arrives, so a duplicate that lands
 * mid-stream or right after stream-end is still rejected. On a playback
 * failure the slot is released so a genuine gateway retry of the same URL can
 * replay. This guard only gates DISPATCH; it never touches STT, mic capture,
 * PCM decode, the stream DSP, or the DMA sizing. */
static SemaphoreHandle_t s_playstream_mtx = NULL;
static char s_playstream_last_url[256] = {0};

static bool playstream_admit(const char *url)
{
    if (url == NULL || url[0] == '\0') {
        return true; /* nothing to key on; let it through (shouldn't happen) */
    }
    bool admit;
    if (s_playstream_mtx != NULL) {
        xSemaphoreTake(s_playstream_mtx, portMAX_DELAY);
    }
    if (s_playstream_last_url[0] != '\0' &&
        strncmp(url, s_playstream_last_url, sizeof(s_playstream_last_url) - 1) == 0) {
        admit = false; /* duplicate of the reply we are already playing/just played */
    } else {
        admit = true;
        strlcpy(s_playstream_last_url, url, sizeof(s_playstream_last_url));
    }
    if (s_playstream_mtx != NULL) {
        xSemaphoreGive(s_playstream_mtx);
    }
    return admit;
}

/* Clear the consumed slot if it still matches `url` (playback failed -> allow a
 * legitimate retry of the same reply to replay). */
static void playstream_release(const char *url)
{
    if (url == NULL) {
        return;
    }
    if (s_playstream_mtx != NULL) {
        xSemaphoreTake(s_playstream_mtx, portMAX_DELAY);
    }
    if (strncmp(url, s_playstream_last_url, sizeof(s_playstream_last_url) - 1) == 0) {
        s_playstream_last_url[0] = '\0';
    }
    if (s_playstream_mtx != NULL) {
        xSemaphoreGive(s_playstream_mtx);
    }
}

static bool get_u8(cJSON *p, const char *key, uint8_t *out, uint8_t def)
{
    cJSON *it = cJSON_GetObjectItemCaseSensitive(p, key);
    if (it == NULL) { *out = def; return true; }
    if (!cJSON_IsNumber(it) || it->valueint < 0 || it->valueint > 255) return false;
    *out = (uint8_t)it->valueint;
    return true;
}

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

/* led.fill { r,g,b } — set all 12 status LEDs. */
static esp_err_t handle_led_fill(
    esp_openclaw_node_handle_t node, void *ctx,
    const char *params_json, size_t params_len,
    char **out_json, esp_openclaw_node_error_t *err)
{
    (void)node; (void)ctx; (void)params_len;
    cJSON *p = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &p, err), TAG, "params");

    uint8_t r, g, b;
    if (!get_u8(p, "r", &r, 0) || !get_u8(p, "g", &g, 0) || !get_u8(p, "b", &b, 0)) {
        err->message = "r, g, b must each be 0-255";
        cJSON_Delete(p);
        return ESP_ERR_INVALID_ARG;
    }
    stackchan_led_fill(r, g, b);

    cJSON *out = cJSON_CreateObject();
    if (!out) { cJSON_Delete(p); return ESP_ERR_NO_MEM; }
    cJSON_AddNumberToObject(out, "r", r);
    cJSON_AddNumberToObject(out, "g", g);
    cJSON_AddNumberToObject(out, "b", b);
    cJSON_Delete(p);
    return esp_openclaw_node_example_take_json_payload(out, out_json);
}

/* display.avatar { on:bool } — start/stop the looping avatar video. */
static esp_err_t handle_display_avatar(
    esp_openclaw_node_handle_t node, void *ctx,
    const char *params_json, size_t params_len,
    char **out_json, esp_openclaw_node_error_t *err)
{
    (void)node; (void)ctx; (void)params_len;
    cJSON *p = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &p, err), TAG, "params");

    cJSON *on = cJSON_GetObjectItemCaseSensitive(p, "on");
    bool enable = (on == NULL) ? true : cJSON_IsTrue(on);
    stackchan_avatar_set(enable);

    cJSON *out = cJSON_CreateObject();
    if (!out) { cJSON_Delete(p); return ESP_ERR_NO_MEM; }
    cJSON_AddBoolToObject(out, "avatar", enable);
    cJSON_Delete(p);
    return esp_openclaw_node_example_take_json_payload(out, out_json);
}

/* display.text { text, size?, r?,g?,b? } — draw centered text (pauses avatar). */
static esp_err_t handle_display_text(
    esp_openclaw_node_handle_t node, void *ctx,
    const char *params_json, size_t params_len,
    char **out_json, esp_openclaw_node_error_t *err)
{
    (void)node; (void)ctx; (void)params_len;
    cJSON *p = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &p, err), TAG, "params");

    cJSON *t = cJSON_GetObjectItemCaseSensitive(p, "text");
    if (!cJSON_IsString(t) || t->valuestring == NULL) {
        err->message = "text (string) required";
        cJSON_Delete(p);
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t size, r, g, b;
    if (!get_u8(p, "size", &size, 3) || !get_u8(p, "r", &r, 255) ||
        !get_u8(p, "g", &g, 255) || !get_u8(p, "b", &b, 255)) {
        err->message = "size/r/g/b must be 0-255";
        cJSON_Delete(p);
        return ESP_ERR_INVALID_ARG;
    }
    stackchan_display_text(t->valuestring, size, rgb565(r, g, b));

    cJSON *out = cJSON_CreateObject();
    if (!out) { cJSON_Delete(p); return ESP_ERR_NO_MEM; }
    cJSON_AddStringToObject(out, "text", t->valuestring);
    cJSON_Delete(p);
    return esp_openclaw_node_example_take_json_payload(out, out_json);
}

/* display.clear { r?,g?,b? } — clear the LCD to a solid color (pauses avatar). */
static esp_err_t handle_display_clear(
    esp_openclaw_node_handle_t node, void *ctx,
    const char *params_json, size_t params_len,
    char **out_json, esp_openclaw_node_error_t *err)
{
    (void)node; (void)ctx; (void)params_len;
    cJSON *p = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &p, err), TAG, "params");

    uint8_t r, g, b;
    if (!get_u8(p, "r", &r, 0) || !get_u8(p, "g", &g, 0) || !get_u8(p, "b", &b, 0)) {
        err->message = "r, g, b must each be 0-255";
        cJSON_Delete(p);
        return ESP_ERR_INVALID_ARG;
    }
    stackchan_display_clear(rgb565(r, g, b));

    cJSON *out = cJSON_CreateObject();
    if (!out) { cJSON_Delete(p); return ESP_ERR_NO_MEM; }
    cJSON_AddBoolToObject(out, "cleared", true);
    cJSON_Delete(p);
    return esp_openclaw_node_example_take_json_payload(out, out_json);
}

/* display.brightness { level } — set LCD backlight 0-255. */
static esp_err_t handle_display_brightness(
    esp_openclaw_node_handle_t node, void *ctx,
    const char *params_json, size_t params_len,
    char **out_json, esp_openclaw_node_error_t *err)
{
    (void)node; (void)ctx; (void)params_len;
    cJSON *p = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &p, err), TAG, "params");

    uint8_t level;
    if (!get_u8(p, "level", &level, 127)) {
        err->message = "level must be 0-255";
        cJSON_Delete(p);
        return ESP_ERR_INVALID_ARG;
    }
    stackchan_display_brightness(level);

    cJSON *out = cJSON_CreateObject();
    if (!out) { cJSON_Delete(p); return ESP_ERR_NO_MEM; }
    cJSON_AddNumberToObject(out, "brightness", level);
    cJSON_Delete(p);
    return esp_openclaw_node_example_take_json_payload(out, out_json);
}

/* speaker.tone { freq, ms?, volume? } — play a tone through the 1W speaker. */
static esp_err_t handle_speaker_tone(
    esp_openclaw_node_handle_t node, void *ctx,
    const char *params_json, size_t params_len,
    char **out_json, esp_openclaw_node_error_t *err)
{
    (void)node; (void)ctx; (void)params_len;
    cJSON *p = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &p, err), TAG, "params");

    cJSON *freq = cJSON_GetObjectItemCaseSensitive(p, "freq");
    if (!cJSON_IsNumber(freq) || freq->valuedouble < 50 || freq->valuedouble > 15000) {
        err->message = "freq (Hz, 50-15000) required";
        cJSON_Delete(p);
        return ESP_ERR_INVALID_ARG;
    }
    cJSON *ms = cJSON_GetObjectItemCaseSensitive(p, "ms");
    uint32_t dur = (cJSON_IsNumber(ms) && ms->valueint > 0) ? (uint32_t)ms->valueint : 300;
    uint8_t vol;
    if (!get_u8(p, "volume", &vol, 128)) {
        err->message = "volume must be 0-255";
        cJSON_Delete(p);
        return ESP_ERR_INVALID_ARG;
    }

    if (!M5.Speaker.isEnabled()) {
        M5.Speaker.begin();
    }
    M5.Speaker.setVolume(vol);
    M5.Speaker.tone((float)freq->valuedouble, dur);

    cJSON *out = cJSON_CreateObject();
    if (!out) { cJSON_Delete(p); return ESP_ERR_NO_MEM; }
    cJSON_AddNumberToObject(out, "freq", freq->valuedouble);
    cJSON_AddNumberToObject(out, "ms", dur);
    cJSON_AddNumberToObject(out, "volume", vol);
    cJSON_Delete(p);
    return esp_openclaw_node_example_take_json_payload(out, out_json);
}

/* speaker.play { audioBase64, sampleRate?, volume? } — play arbitrary raw
 * 16-bit signed PCM (pcm_s16le, mono, little-endian) through the 1W speaker.
 * This is the gateway->device push path for TTS/voice clips: base64-decode the
 * PCM and hand it to the shared half-duplex player (which releases the mic,
 * plays, and blocks until done). Pairs with the auto-pull talk.speak path;
 * either can drive the speaker. */
static esp_err_t handle_speaker_play(
    esp_openclaw_node_handle_t node, void *ctx,
    const char *params_json, size_t params_len,
    char **out_json, esp_openclaw_node_error_t *err)
{
    (void)node; (void)ctx; (void)params_len;
    cJSON *p = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &p, err), TAG, "params");

    cJSON *audio = cJSON_GetObjectItemCaseSensitive(p, "audioBase64");
    if (!cJSON_IsString(audio) || audio->valuestring == NULL || audio->valuestring[0] == '\0') {
        err->message = "audioBase64 (raw pcm_s16le, mono) required";
        cJSON_Delete(p);
        return ESP_ERR_INVALID_ARG;
    }
    cJSON *sr = cJSON_GetObjectItemCaseSensitive(p, "sampleRate");
    uint32_t rate = (cJSON_IsNumber(sr) && sr->valueint >= 8000 && sr->valueint <= 48000)
                        ? (uint32_t)sr->valueint : 16000;
    uint8_t vol;
    if (!get_u8(p, "volume", &vol, 160)) {
        err->message = "volume must be 0-255";
        cJSON_Delete(p);
        return ESP_ERR_INVALID_ARG;
    }

    size_t b64_len = strlen(audio->valuestring);
    size_t pcm_cap = (b64_len / 4) * 3 + 4;
    uint8_t *pcm = (uint8_t *)heap_caps_malloc(pcm_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (pcm == NULL) {
        pcm = (uint8_t *)heap_caps_malloc(pcm_cap, MALLOC_CAP_8BIT);
    }
    if (pcm == NULL) {
        err->message = "PCM alloc failed";
        cJSON_Delete(p);
        return ESP_ERR_NO_MEM;
    }
    size_t pcm_len = 0;
    int rc = mbedtls_base64_decode(pcm, pcm_cap, &pcm_len,
                                   (const uint8_t *)audio->valuestring, b64_len);
    if (rc != 0 || pcm_len < sizeof(int16_t)) {
        free(pcm);
        cJSON_Delete(p);
        err->message = "audioBase64 decode failed";
        return ESP_ERR_INVALID_ARG;
    }

    M5.Speaker.setVolume(vol);
    esp_err_t play_err = stackchan_voice_play_pcm(
        (const int16_t *)pcm, pcm_len / sizeof(int16_t), rate);
    free(pcm);
    if (play_err != ESP_OK) {
        cJSON_Delete(p);
        err->message = "playback failed";
        return play_err;
    }

    cJSON *out = cJSON_CreateObject();
    if (!out) { cJSON_Delete(p); return ESP_ERR_NO_MEM; }
    cJSON_AddNumberToObject(out, "samples", (double)(pcm_len / sizeof(int16_t)));
    cJSON_AddNumberToObject(out, "sampleRate", rate);
    cJSON_AddNumberToObject(out, "volume", vol);
    cJSON_Delete(p);
    return esp_openclaw_node_example_take_json_payload(out, out_json);
}

/* speaker.playStream { url, sampleRate?, volume? } — stream arbitrarily long
 * audio by pulling it from a URL and playing chunks as they arrive (constant
 * memory; safe for 20+ min clips that would OOM the base64 speaker.play path).
 * Body must be raw 16-bit signed PCM (pcm_s16le, mono, little-endian) — the
 * same thing ElevenLabs returns for output_format pcm_16000. The gateway can
 * serve a live/chunked response and the device plays it as it streams. */
#define PLAYSTREAM_BUF 4096
static esp_err_t handle_speaker_play_stream(
    esp_openclaw_node_handle_t node, void *ctx,
    const char *params_json, size_t params_len,
    char **out_json, esp_openclaw_node_error_t *err)
{
    (void)node; (void)ctx; (void)params_len;
    cJSON *p = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &p, err), TAG, "params");

    cJSON *url = cJSON_GetObjectItemCaseSensitive(p, "url");
    if (!cJSON_IsString(url) || url->valuestring == NULL || url->valuestring[0] == '\0') {
        err->message = "url (raw pcm_s16le stream) required";
        cJSON_Delete(p);
        return ESP_ERR_INVALID_ARG;
    }
    cJSON *sr = cJSON_GetObjectItemCaseSensitive(p, "sampleRate");
    uint32_t rate = (cJSON_IsNumber(sr) && sr->valueint >= 8000 && sr->valueint <= 48000)
                        ? (uint32_t)sr->valueint : 16000;
    uint8_t vol;
    if (!get_u8(p, "volume", &vol, 160)) {
        err->message = "volume must be 0-255";
        cJSON_Delete(p);
        return ESP_ERR_INVALID_ARG;
    }

    char *url_copy = strdup(url->valuestring);
    cJSON_Delete(p);
    if (url_copy == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Single-fire guard: the gateway invokes speaker.playStream to play a
     * reply's TTS, and the command is registered on BOTH WS legs under one
     * device identity, so the SAME invocation (identical URL) can be delivered
     * twice (dual-leg fan-out or a gateway retry). Admit the FIRST one and drop
     * any duplicate of the same reply URL so the device never speaks it twice.
     * Compare-and-set is atomic under the guard mutex (no TOCTOU across the two
     * legs' dispatch tasks). */
    if (!playstream_admit(url_copy)) {
        ESP_LOGW(TAG, "playStream duplicate reply dropped leg=%s urltail=%s",
                 stackchan_gateway_leg_name(node), url_tail(url_copy, 44));
        free(url_copy);
        {
            cJSON *dout = cJSON_CreateObject();
            if (dout == NULL) {
                return ESP_ERR_NO_MEM;
            }
            cJSON_AddBoolToObject(dout, "streamed", false);
            cJSON_AddBoolToObject(dout, "deduped", true);
            return esp_openclaw_node_example_take_json_payload(dout, out_json);
        }
    }
    ESP_LOGI(TAG, "playStream reply admitted leg=%s urltail=%s",
             stackchan_gateway_leg_name(node), url_tail(url_copy, 44));

    esp_http_client_config_t cfg = {};
    cfg.url = url_copy;
    cfg.timeout_ms = 15000;
    cfg.crt_bundle_attach = esp_crt_bundle_attach; /* enable https */
    cfg.buffer_size = PLAYSTREAM_BUF;
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        free(url_copy);
        err->message = "http init failed";
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_OK;
    bool stream_open = false;
    uint8_t *buf = (uint8_t *)heap_caps_malloc(PLAYSTREAM_BUF, MALLOC_CAP_8BIT);
    if (buf == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto done;
    }

    /* Reserve the small, FIXED I2S DMA (internal DMA-capable RAM) BEFORE we
     * open the HTTPS/TLS session. mbedTLS's record + handshake buffers grab
     * tens of KB of internal heap during esp_http_client_open(); if that runs
     * first it starves i2s_alloc_dma_desc(), so M5.Speaker.begin() fails
     * ("allocate DMA buffer failed" -> "initialize channel failed") and the TTS
     * reply never plays. Grabbing the scarce fixed DMA first, then letting TLS
     * use whatever remains, fixes the ordering. rate/vol are known from params
     * so we do not need the HTTP response first. */
    if ((ret = stackchan_voice_stream_begin(rate, vol)) != ESP_OK) {
        err->message = "speaker begin failed";
        goto done;
    }
    stream_open = true;

    if ((ret = esp_http_client_open(client, 0)) != ESP_OK) {
        err->message = "http open failed";
        goto done;
    }
    esp_http_client_fetch_headers(client);
    int status;
    status = esp_http_client_get_status_code(client);
    if (status < 200 || status >= 300) {
        ESP_LOGW(TAG, "playStream: HTTP %d", status);
        err->message = "http status not ok";
        ret = ESP_FAIL;
        goto done;
    }

    /* Stream loop. Carry a single leftover byte between reads so int16 samples
     * stay aligned even if a chunk boundary splits a sample. */
    uint8_t carry;
    bool have_carry;
    have_carry = false;
    carry = 0;
    size_t total;
    total = 0;
    for (;;) {
        int off = 0;
        if (have_carry) { buf[0] = carry; off = 1; have_carry = false; }
        int r = esp_http_client_read(client, (char *)buf + off, PLAYSTREAM_BUF - off);
        if (r < 0) { ret = ESP_FAIL; break; }
        int avail = r + off;
        if (avail <= 0) {
            if (esp_http_client_is_complete_data_received(client)) break;
            if (r == 0) break;
            continue;
        }
        if (avail & 1) { carry = buf[avail - 1]; have_carry = true; avail -= 1; }
        if (avail > 0) {
            stackchan_voice_stream_feed((const int16_t *)buf, (size_t)avail / sizeof(int16_t));
            total += (size_t)avail;
        }
        if (r == 0 && esp_http_client_is_complete_data_received(client)) break;
    }
    ESP_LOGI(TAG, "playStream: streamed %u PCM bytes @ %u Hz", (unsigned)total, (unsigned)rate);

done:
    if (stream_open) {
        stackchan_voice_stream_end();
    }
    if (buf) free(buf);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    /* Playback failed: release the consumed URL so a genuine gateway retry of
     * the SAME reply can replay (a duplicate of a SUCCESSFUL play stays
     * dropped because the slot is retained). */
    if (ret != ESP_OK) {
        playstream_release(url_copy);
    }
    free(url_copy);
    if (ret != ESP_OK) {
        if (err->message == NULL) err->message = "stream failed";
        return ret;
    }

    cJSON *out = cJSON_CreateObject();
    if (!out) return ESP_ERR_NO_MEM;
    cJSON_AddNumberToObject(out, "sampleRate", rate);
    cJSON_AddNumberToObject(out, "volume", vol);
    cJSON_AddBoolToObject(out, "streamed", true);
    return esp_openclaw_node_example_take_json_payload(out, out_json);
}

esp_err_t esp_openclaw_node_stackchan_register_body_commands(esp_openclaw_node_handle_t node)
{
    /* Create the playStream single-fire guard mutex once, before any WS leg can
     * dispatch a command. This function runs once per leg at boot (both calls
     * are sequential and single-threaded in app_main), so the NULL check makes
     * the second call a no-op -- both legs then share ONE guard, which is what
     * lets the dedup cover a duplicate delivered across the two legs. */
    if (s_playstream_mtx == NULL) {
        s_playstream_mtx = xSemaphoreCreateMutex();
    }

    static const esp_openclaw_node_command_t CMDS[] = {
        { .name = "led.fill",        .handler = handle_led_fill,       .context = NULL },
        { .name = "display.avatar",  .handler = handle_display_avatar, .context = NULL },
        { .name = "display.text",    .handler = handle_display_text,   .context = NULL },
        { .name = "display.clear",   .handler = handle_display_clear,  .context = NULL },
        { .name = "display.brightness", .handler = handle_display_brightness, .context = NULL },
        { .name = "speaker.tone",    .handler = handle_speaker_tone,   .context = NULL },
        { .name = "speaker.play",    .handler = handle_speaker_play,   .context = NULL },
        { .name = "speaker.playStream", .handler = handle_speaker_play_stream, .context = NULL },
    };

    ESP_RETURN_ON_ERROR(esp_openclaw_node_register_capability(node, "led"), TAG, "cap led");
    ESP_RETURN_ON_ERROR(esp_openclaw_node_register_capability(node, "display"), TAG, "cap display");
    ESP_RETURN_ON_ERROR(esp_openclaw_node_register_capability(node, "speaker"), TAG, "cap speaker");
    for (size_t i = 0; i < sizeof(CMDS) / sizeof(CMDS[0]); i++) {
        ESP_RETURN_ON_ERROR(
            esp_openclaw_node_register_command(node, &CMDS[i]), TAG, "register cmd");
    }
    return ESP_OK;
}
