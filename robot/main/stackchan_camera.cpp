/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * StackChan CoreS3 camera routine: snap (GC0308) -> display on LCD ~3s ->
 * upload the JPEG to OpenClaw for analysis. See stackchan_camera.h for the
 * shared-bus/DMA rationale.
 */

#include "stackchan_camera.h"
#include "stackchan_body.h"        /* stackchan_avatar_set / thinking_set */
#include "stackchan_idle.h"        /* note activity -> reset idle timer / wake */
#include "stackchan_servo.h"       /* stackchan_servo_get_aim -> where the head looks */
#include "stackchan_ready.h"       /* SYS_READY -> one-shot boot capture */
#include <stdio.h>

#include <string.h>

#include <M5Unified.h>

extern "C" {
#include "esp_camera.h"
#include "img_converters.h"
#include "stackchan_voice_net.h"   /* stackchan_voice_net_send_image */
}

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* Legacy power-poll cooperative pause counter (shared with LED/codec paths). */
extern "C" volatile int g_stackchan_power_poll_pause;

/* esp32-camera internal (private_include/cam_hal.h). NOTE: PSRAM DMA mode is
 * DISABLED on purpose. The CoreS3 has *Quad* PSRAM @80MHz whose write
 * bandwidth cannot keep up with the GC0308's fixed external pixel clock, so
 * DMAing straight to PSRAM overflows every frame (cam_hal: EV-VSYNC-OVF ->
 * "Failed to get frame: timeout"). The OOTB M5CoreS3 path uses the non-PSRAM
 * mode: the GDMA fills a small internal line buffer and the driver memcpys
 * each half-buffer into the PSRAM frame buffer, which absorbs the bandwidth
 * mismatch. The frame buffers still live in PSRAM (fb_location=PSRAM); only a
 * small internal DMA line buffer is used, and it is freed on esp_camera_deinit
 * right after each shot so the speaker's I2S DMA is not held. */
extern "C" void cam_set_psram_mode(bool enable);

static const char *TAG = "stackchan_cam";

/* GC0308 SCCB is 100 kHz-safe; keep it modest on the shared bus. */
#define CAM_SCCB_FREQ_HZ 100000

/* ------------------------------------------------------------------ */
/* SCCB transport: every sensor register byte rides M5.In_I2C, the      */
/* single internal-I2C owner. Referenced from the vendored esp32-camera */
/* driver/sccb.c (which deliberately does NOT install a 2nd i2c driver).*/
/* ------------------------------------------------------------------ */
extern "C" int stk_cam_sccb_read8(uint8_t slv_addr, uint8_t reg)
{
    uint8_t v = 0;
    if (!M5.In_I2C.readRegister(slv_addr, reg, &v, 1, CAM_SCCB_FREQ_HZ)) {
        return -1;
    }
    return (int)v;
}

extern "C" int stk_cam_sccb_write8(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    return M5.In_I2C.writeRegister8(slv_addr, reg, data, CAM_SCCB_FREQ_HZ) ? 0 : -1;
}

extern "C" int stk_cam_sccb_probe(uint8_t slv_addr)
{
    return M5.In_I2C.scanID(slv_addr, CAM_SCCB_FREQ_HZ) ? 0 : -1;
}

/* ------------------------------------------------------------------ */
/* Camera config (M5Stack CoreS3 GC0308, from the M5CoreS3 OOTB driver) */
/* ------------------------------------------------------------------ */
/* pin_xclk = -1: the CoreS3 GC0308 is clocked externally on-board, so the
 * MCU does not drive XCLK (verified against M5CoreS3/src/utility/GC0308.cpp).
 * SCCB pins are left as -1 and sccb_i2c_port set so esp_camera takes the
 * "use existing I2C port" path -> our no-op SCCB_Use_Port -> all sensor I/O
 * goes through M5.In_I2C. RGB565 QVGA (320x240) fills the 320x240 LCD 1:1 and
 * software-encodes to a small JPEG for upload. Frame buffers in PSRAM. */
static camera_config_t make_cam_cfg(framesize_t fs, pixformat_t pf)
{
    camera_config_t c;
    memset(&c, 0, sizeof(c));
    c.pin_pwdn      = -1;
    c.pin_reset     = -1;
    c.pin_xclk      = -1;
    c.pin_sccb_sda  = -1;   /* -> SCCB_Use_Port path (M5 owns the bus) */
    c.pin_sccb_scl  = -1;
    c.pin_d7 = 47;
    c.pin_d6 = 48;
    c.pin_d5 = 16;
    c.pin_d4 = 15;
    c.pin_d3 = 42;
    c.pin_d2 = 41;
    c.pin_d1 = 40;
    c.pin_d0 = 39;
    c.pin_vsync = 46;
    c.pin_href  = 38;
    c.pin_pclk  = 45;
    c.xclk_freq_hz = 20000000;
    c.ledc_timer   = LEDC_TIMER_0;
    c.ledc_channel = LEDC_CHANNEL_0;
    c.pixel_format = pf;
    c.frame_size   = fs;
    c.jpeg_quality = 0;                /* unused for RGB565 sensor capture */
    c.fb_count     = 2;                /* 2 lets us grab a settled 2nd frame cheaply */
    c.fb_location  = CAMERA_FB_IN_PSRAM;
    c.grab_mode    = CAMERA_GRAB_LATEST;
    c.sccb_i2c_port = 1;               /* CoreS3 internal bus is I2C_NUM_1 (M5-owned) */
    return c;
}

/* ------------------------------------------------------------------ */
/* Single-flight guard + PERSISTENT worker.                             */
/* ------------------------------------------------------------------ */
/* A capture is a multi-second display+upload; a second trigger (e.g.
 * camera.capture delivered on BOTH WS legs, or a serial `snap` on top of a
 * gateway trigger) must not run concurrently -- two esp_camera_init()s on one
 * sensor would collide. Mirror the playStream single-fire discipline: atomic
 * test-and-set under a mutex.
 *
 * The worker is a PERSISTENT task created LAZILY on the FIRST actual capture
 * (not at boot) that blocks on a trigger semaphore. Once created it is kept
 * for the life of the device, so it is a create-once, not per-capture, task
 * (per-capture xTaskCreate was unreliable under TLS-reconnect heap frag). It
 * is deferred to first use because holding the 8 KB internal task stack from
 * boot starved the contiguous internal-DMA pool needed for the two concurrent
 * WS/TLS handshakes -> the node leg could not connect. By the time a capture
 * is requested BOTH legs are up, the reconnect storm has stopped, and the
 * internal heap has recovered enough to allocate the stack. */
static SemaphoreHandle_t s_cam_mtx = NULL;      /* guards s_capturing + prompt */
static SemaphoreHandle_t s_cam_trigger = NULL;  /* binary: fire a capture */
static bool s_capturing = false;
static char s_pending_prompt[256] = {0};

static bool capture_admit(void)
{
    bool admit = false;
    if (s_cam_mtx) xSemaphoreTake(s_cam_mtx, portMAX_DELAY);
    if (!s_capturing) { s_capturing = true; admit = true; }
    if (s_cam_mtx) xSemaphoreGive(s_cam_mtx);
    return admit;
}

static void capture_release(void)
{
    if (s_cam_mtx) xSemaphoreTake(s_cam_mtx, portMAX_DELAY);
    s_capturing = false;
    if (s_cam_mtx) xSemaphoreGive(s_cam_mtx);
}

/* ------------------------------------------------------------------ */
/* The snap -> display -> upload cycle (runs on the persistent worker). */
/* ------------------------------------------------------------------ */
static const char DEFAULT_PROMPT[] =
    "This is a live photo from my onboard camera -- my actual view of the room right now. "
    "Silently update your understanding of my surroundings from it (people, setting, mood, "
    "what's going on), then say ONE short, natural, in-character line that subtly shows you "
    "can see -- a casual remark about the scene, not a description. "
    "Example vibe: 'Ooh, looks like you two are having a movie night.' "
    "Don't narrate that you're analyzing a photo, don't list what you see, don't mention the "
    "camera. Just react like you're in the room. "
    "The image is grayscale only to save memory -- do NOT comment on it being black-and-white, "
    "monochrome, or artsy; treat it as a normal full-color scene.";

static void camera_run_once(const char *prompt)
{
    esp_err_t err = ESP_OK;
    camera_fb_t *fb = NULL;
    uint8_t *jpg = NULL;
    size_t jpg_len = 0;
    bool cam_up = false;

    int64_t t0 = esp_timer_get_time();
    ESP_LOGI(TAG, "pre-init heap: DMA largest-free-block=%u total-DMA-free=%u internal-free=%u",
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    /* Pause the AXP2101 power-key poller across the SCCB-heavy init/deinit
     * window (same cooperative discipline as the LED/codec paths). Sensor
     * register I/O is serialized behind M5.In_I2C's own lock regardless; this
     * just avoids a needless poll landing mid-burst. A settle lets any in-
     * flight bus transaction finish before we start the camera. */
    g_stackchan_power_poll_pause++;
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Non-PSRAM DMA (internal line buffer + memcpy to the PSRAM frame buffer).
     * See the cam_set_psram_mode note above for why direct-to-PSRAM overflows
     * on CoreS3's Quad PSRAM. Set explicitly (don't rely on the Kconfig default)
     * before esp_camera_init reads g_psram_dma_mode in cam_config(). */
    cam_set_psram_mode(false);

    /* The internal DMA line buffer (~7.7 KB at QVGA, ~3.8 KB at QQVGA) is the
     * scarce resource: this firmware runs the internal DMA heap very close to
     * full, and a gateway TLS (re)connect transiently grabs big internal
     * buffers. So: (1) retry a few times per size to ride out a transient dip,
     * and (2) if the larger size never fits, fall back to a smaller one so the
     * user still gets a photo. All modes are RGB565 to keep display + JPEG
     * encode uniform. */
    {
        /* min_dma = the contiguous internal DMA the mode's line buffer needs
         * (2 x half-buffer) plus a little slack for descriptors. Pre-checking
         * the largest free DMA block avoids ~0.5s of SCCB/XCLK churn per doomed
         * esp_camera_init when the internal DMA heap is exhausted (which it is
         * whenever the gateway is unreachable and the WS legs storm TLS
         * reconnects -- each handshake transiently grabs the internal DMA). */
        static const struct { framesize_t fs; pixformat_t pf; size_t min_dma; const char *name; } MODES[] = {
            { FRAMESIZE_QVGA,  PIXFORMAT_RGB565,    8192, "QVGA 320x240 RGB565 (~7.7KB DMA)" },
            { FRAMESIZE_QQVGA, PIXFORMAT_RGB565,    4096, "QQVGA 160x120 RGB565 (~3.8KB DMA)" },
            { FRAMESIZE_QQVGA, PIXFORMAT_GRAYSCALE, 2200, "QQVGA 160x120 GRAY (~1.9KB DMA)" },
        };
        const int RETRIES_PER_MODE = 4;
        for (size_t m = 0; m < sizeof(MODES) / sizeof(MODES[0]) && !cam_up; m++) {
            camera_config_t cfg = make_cam_cfg(MODES[m].fs, MODES[m].pf);
            for (int attempt = 0; attempt < RETRIES_PER_MODE; attempt++) {
                size_t big = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
                if (big < MODES[m].min_dma) {
                    vTaskDelay(pdMS_TO_TICKS(250)); /* wait for a DMA dip to pass */
                    continue;
                }
                err = esp_camera_init(&cfg);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "camera init ok @ %s (attempt %d)", MODES[m].name, attempt + 1);
                    cam_up = true;
                    break;
                }
                esp_camera_deinit(); /* clean any partial state before retry */
                vTaskDelay(pdMS_TO_TICKS(250));
            }
            if (!cam_up) {
                ESP_LOGW(TAG, "camera: %s did not fit (DMA largest-free-block=%u); trying smaller",
                         MODES[m].name, (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
            }
        }
    }
    if (!cam_up) {
        ESP_LOGE(TAG, "camera: no mode fit the free internal DMA (largest-free-block=%u). "
                      "This happens while the gateway is unreachable and the WS legs storm TLS "
                      "reconnects, which exhausts internal DMA. Retry once connected/idle.",
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
        err = ESP_FAIL;
        if (g_stackchan_power_poll_pause > 0) g_stackchan_power_poll_pause--;
        goto done;
    }

    /* Warm-up: the first frame after init is often under-exposed while the
     * sensor's auto-exposure/AWB settle. Grab and drop one, wait briefly, then
     * grab the frame we actually use. */
    fb = esp_camera_fb_get();
    if (fb) { esp_camera_fb_return(fb); fb = NULL; }
    vTaskDelay(pdMS_TO_TICKS(250));

    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "esp_camera_fb_get returned NULL");
        err = ESP_FAIL;
        if (g_stackchan_power_poll_pause > 0) g_stackchan_power_poll_pause--;
        goto done;
    }
    ESP_LOGI(TAG, "captured frame: %ux%u fmt=%d len=%u bytes",
             (unsigned)fb->width, (unsigned)fb->height, (int)fb->format, (unsigned)fb->len);

    /* --- On-screen preview REMOVED ---
     * Previously the frame was blitted to the LCD and held ~3s, which fought the
     * avatar / presentation / idle layers and kept the capture worker busy long
     * enough to reject rapid re-triggers. We now go straight from capture ->
     * JPEG -> upload and never touch the display. */

    /* --- Encode to JPEG for upload (software; GC0308 has no HW JPEG). --- */
    if (!fmt2jpg(fb->buf, fb->len, fb->width, fb->height,
                 fb->format, 80, &jpg, &jpg_len) || jpg == NULL) {
        ESP_LOGE(TAG, "fmt2jpg failed");
        err = ESP_FAIL;
        jpg = NULL;
    } else {
        ESP_LOGI(TAG, "encoded JPEG: %u bytes", (unsigned)jpg_len);
    }

    /* Frame buffer + camera are no longer needed: return the fb and deinit so
     * the camera holds ZERO DMA/PSRAM between shots (protects the speaker's
     * scarce internal DMA). The pushed image remains on the LCD. */
    esp_camera_fb_return(fb);
    fb = NULL;
    esp_camera_deinit();
    cam_up = false;
    if (g_stackchan_power_poll_pause > 0) g_stackchan_power_poll_pause--;

    /* --- Upload to OpenClaw for analysis (node.event agent.request). --- */
    if (jpg != NULL && jpg_len > 0) {
        /* Append where the head is pointing so the bot understands the framing
         * of this shot (best-effort last-commanded aim). yaw -left/+right,
         * pitch -down/+up. Turned into a plain-English hint + raw degrees. */
        int aim_yaw = 0, aim_pitch = 0;
        stackchan_servo_get_aim(&aim_yaw, &aim_pitch);
        const char *h_dir =
            (aim_yaw <= -30) ? "far to your left" :
            (aim_yaw <=  -8) ? "to your left" :
            (aim_yaw >=  30) ? "far to your right" :
            (aim_yaw >=   8) ? "to your right" : "straight ahead";
        const char *v_dir =
            (aim_pitch >=  20) ? ", angled up" :
            (aim_pitch >=   6) ? ", tilted up a little" :
            (aim_pitch <= -20) ? ", angled down" :
            (aim_pitch <=  -6) ? ", tilted down a little" : "";
        const char *base = (prompt && prompt[0]) ? prompt : DEFAULT_PROMPT;
        char full_prompt[1024];
        snprintf(full_prompt, sizeof(full_prompt),
                 "%s (Camera framing: my head is looking %s%s -- head yaw %d deg, "
                 "pitch %d deg, where yaw is +right/-left and pitch is +up/-down. "
                 "Use this to reason about where things are relative to me.)",
                 base, h_dir, v_dir, aim_yaw, aim_pitch);

        esp_err_t up = stackchan_voice_net_send_image(full_prompt, jpg, jpg_len);
        if (up == ESP_OK) {
            ESP_LOGI(TAG, "image handed to OpenClaw (agent.request) ok");
        } else {
            ESP_LOGW(TAG, "image upload submit failed: %s", esp_err_to_name(up));
        }
    }

    ESP_LOGI(TAG, "capture cycle done in %lld ms",
             (long long)((esp_timer_get_time() - t0) / 1000));

done:
    if (fb) esp_camera_fb_return(fb);
    if (cam_up) {
        esp_camera_deinit();
        if (g_stackchan_power_poll_pause > 0) g_stackchan_power_poll_pause--;
    }
    if (jpg) free(jpg);
    /* Nothing to restore on the display: the capture path no longer touches it. */
}

/* Persistent worker: wait for a trigger, run one capture cycle, release the
 * single-flight guard, repeat. Never exits. */
static void camera_worker(void *arg)
{
    (void)arg;
    for (;;) {
        xSemaphoreTake(s_cam_trigger, portMAX_DELAY);
        char prompt[sizeof(s_pending_prompt)];
        if (s_cam_mtx) xSemaphoreTake(s_cam_mtx, portMAX_DELAY);
        strlcpy(prompt, s_pending_prompt, sizeof(prompt));
        if (s_cam_mtx) xSemaphoreGive(s_cam_mtx);
        camera_run_once(prompt[0] ? prompt : DEFAULT_PROMPT);
        capture_release();
    }
}

/* Bring up the persistent worker + sync primitives exactly once. */
static void camera_ensure_started(void)
{
    static bool started = false;
    if (s_cam_mtx == NULL) {
        s_cam_mtx = xSemaphoreCreateMutex();
    }
    if (s_cam_trigger == NULL) {
        s_cam_trigger = xSemaphoreCreateBinary();
    }
    if (!started && s_cam_trigger != NULL) {
        /* 8 KB stack (fmt2jpg + M5.Display push are the heavy users), placed in
         * PSRAM (MALLOC_CAP_SPIRAM); the TCB stays internal (the WithCaps API
         * forces TCBs internal). Two reasons:
         *  1. Created LAZILY on first capture (not at boot) so it holds ZERO
         *     internal SRAM at boot/idle -> the two WS/TLS handshakes keep the
         *     contiguous internal-DMA the node leg needs to connect.
         *  2. Stack in PSRAM (not internal) so creation SUCCEEDS even though the
         *     internal heap is fragmented at steady state (largest contiguous
         *     internal block sits ~4-8 KB once both legs + WireGuard are up, too
         *     small for a plain internal 8 KB task stack). The cam worker does
         *     no TLS and no flash-cache-disabling work on its stack, so a PSRAM
         *     stack is safe here. Core 0 -- core 1 hosts the avatar loop. */
        if (xTaskCreatePinnedToCoreWithCaps(camera_worker, "cam_worker", 8192, NULL, 4, NULL, 0,
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) == pdPASS) {
            started = true;
        } else {
            ESP_LOGE(TAG, "failed to create persistent camera worker");
        }
    }
}

bool stackchan_camera_capture_async(const char *prompt)
{
    /* A capture is activity: reset the idle timers + wake from light idle so the
     * photo displays on a live screen. */
    stackchan_idle_note_activity();
    camera_ensure_started();
    if (s_cam_trigger == NULL) {
        ESP_LOGE(TAG, "camera worker not available");
        return false;
    }
    if (!capture_admit()) {
        ESP_LOGW(TAG, "capture already in flight; ignoring trigger");
        return false;
    }
    /* Stage the prompt for the worker, then fire the trigger. */
    if (s_cam_mtx) xSemaphoreTake(s_cam_mtx, portMAX_DELAY);
    if (prompt && prompt[0]) {
        strlcpy(s_pending_prompt, prompt, sizeof(s_pending_prompt));
    } else {
        s_pending_prompt[0] = '\0';
    }
    if (s_cam_mtx) xSemaphoreGive(s_cam_mtx);
    xSemaphoreGive(s_cam_trigger);
    return true;
}

/* ------------------------------------------------------------------ */
/* Node command: camera.capture { prompt? }                            */
/* ------------------------------------------------------------------ */
extern "C" {
#include "cJSON.h"
#include "esp_openclaw_node_example_json.h"
}

static esp_err_t handle_camera_capture(
    esp_openclaw_node_handle_t node, void *ctx,
    const char *params_json, size_t params_len,
    char **out_json, esp_openclaw_node_error_t *err)
{
    (void)node; (void)ctx; (void)params_len;
    cJSON *p = NULL;
    /* params are optional for camera.capture; tolerate empty/missing. */
    if (params_json && params_json[0]) {
        p = cJSON_Parse(params_json);
    }
    const char *prompt = NULL;
    if (p) {
        cJSON *pr = cJSON_GetObjectItemCaseSensitive(p, "prompt");
        if (cJSON_IsString(pr) && pr->valuestring && pr->valuestring[0]) {
            prompt = pr->valuestring;
        }
    }

    bool started = stackchan_camera_capture_async(prompt);

    cJSON *out = cJSON_CreateObject();
    if (!out) { if (p) cJSON_Delete(p); return ESP_ERR_NO_MEM; }
    cJSON_AddBoolToObject(out, "started", started);
    cJSON_AddBoolToObject(out, "busy", !started);
    cJSON_AddStringToObject(out, "note",
        started ? "snapping; photo shown on LCD ~3s then sent for analysis"
                : "a capture is already in progress");
    if (p) cJSON_Delete(p);
    (void)err;
    return esp_openclaw_node_example_take_json_payload(out, out_json);
}

esp_err_t stackchan_camera_register_commands(esp_openclaw_node_handle_t node)
{
    /* NOTE: do NOT camera_ensure_started() here. The 8 KB worker task stack is
     * created lazily on the first capture (see camera_ensure_started) so it
     * costs ZERO internal SRAM at boot/idle, leaving the contiguous internal-
     * DMA pool free for the operator+node WS/TLS handshakes. */
    esp_err_t err = esp_openclaw_node_register_capability(node, "camera");
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "register capability 'camera' failed: %s", esp_err_to_name(err));
    }
    static const esp_openclaw_node_command_t CMD = {
        .name = "camera.capture", .handler = handle_camera_capture, .context = NULL
    };
    return esp_openclaw_node_register_command(node, &CMD);
}

/* ------------------------------------------------------------------ */
/* Serial console: `camera test` / `snap` (works without the gateway;   */
/* upload is a no-op if not connected, but snap+display still verify).   */
/* ------------------------------------------------------------------ */
#include "esp_console.h"

static int handle_camera_console(int argc, char **argv)
{
    const char *sub = (argc >= 2) ? argv[1] : "test";
    if (strcmp(sub, "test") == 0 || strcmp(sub, "snap") == 0) {
        bool started = stackchan_camera_capture_async(NULL);
        printf("camera: capture %s\n", started ? "started" : "busy (already running)");
        return 0;
    }
    printf("usage: camera test | snap\n");
    return 0;
}

/* ------------------------------------------------------------------ */
/* One-shot boot capture: snap once the first time we reach SYS_READY.  */
/* ------------------------------------------------------------------ */
static const char BOOT_PROMPT[] =
    "I just powered on -- this is the first live look through my onboard camera "
    "for this session. Silently take in my surroundings (who's here, the setting, "
    "the vibe), then greet me with ONE short, natural, in-character line that "
    "subtly shows you can see where we are. Don't narrate that you're analyzing a "
    "photo, don't list what you see, don't mention the camera -- just say hi like "
    "you just looked up and noticed the room. "
    "The image is grayscale only to save memory -- do NOT comment on it being black-and-white, "
    "monochrome, or artsy; treat it as a normal full-color scene.";

/* Give the post-connect TLS/DMA storm a moment to pass before grabbing the
 * camera (which needs contiguous internal DMA). */
static const uint64_t BOOT_CAPTURE_DELAY_US = 3000000ULL; /* 3s */

static void boot_capture_timer_cb(void *arg)
{
    (void)arg;
    bool started = stackchan_camera_capture_async(BOOT_PROMPT);
    ESP_LOGI(TAG, "boot capture: fired one-shot startup photo (started=%d)", (int)started);
}

static void boot_capture_on_ready(stackchan_ready_state_t state, void *ctx)
{
    (void)ctx;
    static bool armed = false; /* fire at most once per boot */
    if (armed || state != SYS_READY) return;
    armed = true;
    esp_timer_handle_t t = NULL;
    const esp_timer_create_args_t args = {
        .callback = boot_capture_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "boot_cam",
        .skip_unhandled_events = false,
    };
    if (esp_timer_create(&args, &t) == ESP_OK) {
        esp_timer_start_once(t, BOOT_CAPTURE_DELAY_US);
        ESP_LOGI(TAG, "boot capture: armed (fires in %llu ms after first READY)",
                 (unsigned long long)(BOOT_CAPTURE_DELAY_US / 1000));
    } else {
        ESP_LOGW(TAG, "boot capture: failed to create one-shot timer");
    }
}

void stackchan_camera_arm_boot_capture(void)
{
    /* If we're somehow already READY (fast boot), the callback still fires on the
     * next transition; also handle the already-ready case directly. */
    esp_err_t e = stackchan_ready_on_change(boot_capture_on_ready, NULL);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "boot capture: could not subscribe to ready changes: %s",
                 esp_err_to_name(e));
        return;
    }
    if (stackchan_ready_get() == SYS_READY) {
        boot_capture_on_ready(SYS_READY, NULL);
    }
}

esp_err_t stackchan_camera_register_console(void)
{
    /* No camera_ensure_started() here either -- the worker is created lazily on
     * the first `camera test`/`snap` (see stackchan_camera_capture_async). */
    esp_console_cmd_t cmd = {};
    cmd.command = "camera";
    cmd.help = "Snap a photo from the CoreS3 camera, show it on the LCD, and send it to OpenClaw.";
    cmd.hint = "test|snap";
    cmd.func = handle_camera_console;
    esp_err_t e = esp_console_cmd_register(&cmd);
    /* Also register bare `snap` as a friendly alias. */
    esp_console_cmd_t alias = {};
    alias.command = "snap";
    alias.help = "Alias for `camera snap`.";
    alias.func = handle_camera_console;
    esp_console_cmd_register(&alias);
    return e;
}
