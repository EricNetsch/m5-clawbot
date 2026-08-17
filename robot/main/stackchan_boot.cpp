/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "stackchan_boot.h"
#include "stackchan_body.h"
#include "esp_openclaw_node_stackchan_rgb_cmd.h" /* g_stackchan_power_poll_pause */
#include "stackchan_ready.h"                     /* system-readiness -> LCD dot */
#include "stackchan_media.h"                     /* swappable frame source      */
#include "stackchan_presentation.h"              /* effective LCD state per state */

#include <M5Unified.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "stackchan_boot";

/* PY32 IO-expander on the robot base (drives the 12x WS2812C LEDs).
 * M5.begin() has already powered the base (AXP2101 + AW9523 boost/bus),
 * so we can talk to the PY32 directly over M5Unified's internal I2C. */
static constexpr uint8_t PY32_ADDR = 0x6F;
static constexpr uint32_t PY32_FREQ = 100000;
static constexpr uint8_t REG_GPIO_M_L = 0x03;  /* dir low  (pin0)  */
static constexpr uint8_t REG_GPIO_M_H = 0x04;  /* dir high (pin13) */
static constexpr uint8_t REG_GPIO_O_L = 0x05;  /* output low       */
static constexpr uint8_t REG_GPIO_PU_L = 0x09; /* pull-up low      */
static constexpr uint8_t REG_GPIO_PU_H = 0x0A; /* pull-up high     */
static constexpr uint8_t REG_GPIO_DRV_H = 0x14;/* drive high       */
static constexpr uint8_t REG_LED_CFG = 0x24;   /* count + refresh  */
static constexpr uint8_t REG_LED_RAM = 0x30;   /* RGB565, lo first */
static constexpr uint8_t LED_COUNT = 12;
static constexpr uint8_t VM_EN_PIN = 0;
static constexpr uint8_t RGB_PIN = 13;

/* Avatar video: JPEG frames in the SPIFFS "storage" partition. */
static constexpr int AVATAR_FRAME_COUNT = 70;  /* f001.jpg .. f070.jpg */
static constexpr int AVATAR_FRAME_W = 320;
static constexpr int AVATAR_FRAME_H = 240;
/* Per-state playback rate now comes from the media provider (stackchan_media). */

/* When false, the avatar task stops drawing so static content can own the LCD. */
static volatile bool s_avatar_playing = true;

/* ------------------------------------------------------------------ */
/* System-readiness status dot (top-right corner of the LCD).          */
/* Drawn as an OVERLAY at the END of each full-screen draw so it isn't  */
/* clobbered by the avatar frame beneath it. LCD is SPI (M5.Display),   */
/* OFF the shared internal I2C bus -- drawing it never touches the      */
/* AW88298/ES7210 codecs or the AXP2101 power poller, so it is safe to  */
/* draw from the avatar/thinking loops at any time. DO NOT add any I2C  */
/* traffic here. */
static constexpr int DOT_R = 5;      /* dot radius */
static constexpr int DOT_MARGIN = 9; /* inset from the top-right corner */
/* Width of the right-edge strip we EXCLUDE from the avatar JPEG blit so the
 * dot is never overwritten (see avatar_video_task for the anti-flicker
 * rationale). Must fully contain the dot: dot left edge = width()-DOT_MARGIN-
 * (DOT_R+2). With DOT_MARGIN=9,DOT_R=5 that's width()-16, so a 20px strip
 * covers it with margin. The avatar icon is centred on a black background so
 * this rightmost strip is black in every frame -> freezing it is invisible. */
static constexpr int DOT_STRIP_W = 20;

/* Draw the dot into any GFX target (the live SPI display OR an off-screen
 * PSRAM sprite/canvas). Templated so the same code composites into the
 * double-buffered avatar canvas (preferred, flicker-free) and still works if we
 * ever fall back to drawing straight to M5.Display. The dot is positioned
 * relative to the target's own width(): for the 320x240 canvas that lands at
 * the same top-right corner as the 320x240 display once pushed at (0,0). */
template <typename GFX>
static void draw_status_dot(GFX &gfx)
{
    const int cx = gfx.width() - DOT_MARGIN;
    const int cy = DOT_MARGIN;

    /* Per Eric: once fully READY, show NO dot -- the indicator only signals the
     * transient/attention states (grey=BOOTING, amber=CONNECTING) and the error
     * state (red=DEGRADED). At READY we clear the dot's spot back to black so no
     * stale colour lingers (the avatar bg here is black; the strip is frozen). */
    if (stackchan_ready_get() == SYS_READY) {
        gfx.fillCircle(cx, cy, DOT_R + 2, gfx.color888(0, 0, 0));
        return;
    }

    uint32_t color;
    switch (stackchan_ready_get()) {
        case SYS_CONNECTING: color = gfx.color888(255, 176, 0);  break; /* amber  */
        case SYS_DEGRADED:   color = gfx.color888(230, 40, 40);  break; /* red    */
        case SYS_BOOTING:
        default:             color = gfx.color888(90, 90, 90);   break; /* grey   */
    }
    /* Subtle dark outline so the dot reads over any avatar pixels. */
    gfx.fillCircle(cx, cy, DOT_R + 2, gfx.color888(0, 0, 0));
    gfx.fillCircle(cx, cy, DOT_R, color);
}

void stackchan_avatar_set(bool playing)
{
    s_avatar_playing = playing;
}

void stackchan_led_fill(uint8_t r, uint8_t g, uint8_t b)
{
    auto &i2c = M5.In_I2C;
    uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    uint8_t data[2] = { (uint8_t)(c & 0xFF), (uint8_t)((c >> 8) & 0xFF) };
    /* Pause the AXP2101 power-key poller for the duration of this LED burst.
     * The WS2812 ring hangs off the PY32 expander on M5.In_I2C -- the SAME
     * physical internal I2C bus the poller reads INTSTS on via a SEPARATE
     * i2c_master stack. Without this, even a single static-color burst races
     * the poller's 40ms read -> the poller's i2c_master_transmit(1207) fails ->
     * the wedged bus makes it misread INTSTS1 bit5 -> false "power key SHORT
     * press -> graceful shutdown". Yielding the bus (the same mechanism the
     * audio codec begin/end uses) removes the last LED-vs-poller collision, so
     * the confirm-gated thinking LED can light without tripping a false
     * shutdown. A missed poll is harmless (PKEY IRQs latch W1C). The short
     * settle lets any in-flight poll read finish before we drive the bus. */
    g_stackchan_power_poll_pause++;
    vTaskDelay(pdMS_TO_TICKS(5));
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        i2c.writeRegister(PY32_ADDR, REG_LED_RAM + i * 2, data, 2, PY32_FREQ);
    }
    i2c.bitOn(PY32_ADDR, REG_LED_CFG, 0x40, PY32_FREQ);
    if (g_stackchan_power_poll_pause > 0) {
        g_stackchan_power_poll_pause--;
    }
}

/* ------------------------------------------------------------------ */
/* THINKING / waiting-for-reply state: static pink LED ring + the      */
/* dedicated jem-thinking clip (t001..t050.jpg) played by the normal   */
/* avatar render loop. Runs from PTT-release until the reply audio     */
/* starts streaming back.                                              */
/*                                                                     */
/* NOTE (history): THINKING used to hard-own the LCD with a pulsing    */
/* pink orb drawn by this task, with the avatar loop PAUSED. It now     */
/* leaves the avatar loop RUNNING; because the presentation coordinator */
/* reports STK_PRES_THINKING while s_thinking is set, the media         */
/* provider serves the t-frames and the render loop blits them like any */
/* other state. This task's ONLY jobs now are (1) light the static     */
/* pink LED once and (2) run the no-reply watchdog. It draws NOTHING to */
/* the LCD, so the fragile Speaker.begin window is untouched by this    */
/* task (see also the dot-suppression for THINKING in the avatar loop).*/
/* ------------------------------------------------------------------ */
static volatile bool s_thinking = false;
static TaskHandle_t s_thinking_task = NULL;

/* Base pink color at full brightness (R,G,B). */
static constexpr uint8_t THINK_R = 255;
static constexpr uint8_t THINK_G = 18;
static constexpr uint8_t THINK_B = 96;

static void thinking_task(void *arg)
{
    (void)arg;

    uint32_t t = 0;
    const uint32_t max_frames = 25 * 25; /* ~25s watchdog: never hang on a
                                          * text-only/failed reply */

    /* Light the LED ring ONCE (static pink), then NEVER touch it again while
     * thinking. The WS2812 ring is driven over M5.In_I2C (PY32 expander) -- the
     * SAME internal I2C bus as the AW88298/ES7210 audio codecs and the AXP2101
     * power chip. The old ~25fps per-frame stackchan_led_fill() hammered that
     * shared bus (~325 I2C txns/s), which wedged it during the reply: the
     * speaker codec begin() failed (reply never played) AND the AXP2101 power
     * poller misread the contended bus as a "power key SHORT press -> graceful
     * shutdown" storm (servos went limp). We light one quiet pink burst and
     * leave it; the visible "thinking" motion is now the t-frame avatar clip
     * (M5.Display = SPI, off the shared I2C bus). */
    stackchan_led_fill((uint8_t)(THINK_R * 0.7f), (uint8_t)(THINK_G * 0.7f),
                       (uint8_t)(THINK_B * 0.7f));

    /* Watchdog only. The avatar loop keeps drawing the t-frames; this task
     * touches NO hardware except the single LED burst above. Do NOT draw to
     * the LCD here and do NOT pause the avatar loop -- that is what lets the
     * jem-thinking clip play during THINKING instead of the old pink orb. */
    while (s_thinking && t < max_frames) {
        t++;
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    /* Self-timeout fallback: drop back to a resting look so we never sit in
     * pink forever if no reply audio ever comes. The avatar loop is already
     * running, so clearing s_thinking flips the presentation state back to
     * READY and the idle f-frames resume on their own. */
    if (t >= max_frames && s_thinking) {
        s_thinking = false;
        stackchan_led_fill(12, 2, 0); /* faint amber = idle */
    }
    s_thinking_task = NULL;
    vTaskDelete(NULL);
}

void stackchan_thinking_set(bool on)
{
    if (on) {
        if (s_thinking) { return; }
        /* Leave the avatar loop RUNNING: the presentation coordinator now
         * reports STK_PRES_THINKING (s_thinking is set), so the render loop
         * switches to the t-frame clip at the next frame boundary. */
        s_thinking = true;
        xTaskCreatePinnedToCore(thinking_task, "thinking", 4096, NULL, 4,
                                &s_thinking_task, 1);
    } else {
        if (!s_thinking) { return; }
        s_thinking = false;
        /* Wait for the watchdog task to exit before returning. */
        for (int i = 0; i < 20 && s_thinking_task != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

bool stackchan_thinking_active(void)
{
    return s_thinking;
}

void stackchan_display_clear(uint16_t color565)
{
    s_avatar_playing = false;
    vTaskDelay(pdMS_TO_TICKS(120));  /* let the avatar task finish a frame */
    M5.Display.fillScreen(color565);
}

void stackchan_display_text(const char *text, uint8_t size, uint16_t color565)
{
    s_avatar_playing = false;
    vTaskDelay(pdMS_TO_TICKS(120));
    auto &lcd = M5.Display;
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextColor(color565, TFT_BLACK);
    lcd.setTextDatum(middle_center);
    lcd.setTextSize(size < 1 ? 1 : size);
    lcd.drawString(text, lcd.width() / 2, lcd.height() / 2);
}

void stackchan_display_goodnight(void)
{
    /* Shutdown "goodnight" screen. LCD is SPI -> safe to draw from the power
     * button task; adds NO I2C traffic that could collide with the AXP2101
     * poller. Make sure nothing else owns the LCD first: stop the thinking orb
     * (if somehow active) and release the avatar loop, then let any in-flight
     * frame finish before we draw. */
    s_thinking = false;
    s_avatar_playing = false;
    vTaskDelay(pdMS_TO_TICKS(140));

    auto &lcd = M5.Display;

    /* Prefer a custom /spiffs/goodnight.jpg if one is present (still SPI). */
    FILE *f = fopen("/spiffs/goodnight.jpg", "rb");
    if (f != NULL) {
        bool drawn = false;
        const size_t cap = 32 * 1024;
        uint8_t *buf = (uint8_t *)malloc(cap);
        if (buf != NULL) {
            size_t n = fread(buf, 1, cap, f);
            if (n > 0 && n < cap) {
                lcd.fillScreen(TFT_BLACK);
                lcd.drawJpg(buf, n, (lcd.width() - AVATAR_FRAME_W) / 2,
                            (lcd.height() - AVATAR_FRAME_H) / 2);
                drawn = true;
            }
            free(buf);
        }
        fclose(f);
        if (drawn) { return; }
    }

    /* Fallback: centered "Goodnight" text on black. */
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextColor(lcd.color888(180, 200, 255), TFT_BLACK);
    lcd.setTextDatum(middle_center);
    lcd.setTextSize(3);
    lcd.drawString("Goodnight", lcd.width() / 2, lcd.height() / 2);
}

void stackchan_display_brightness(uint8_t level)
{
    M5.Display.setBrightness(level);
}

void stackchan_set_volume(uint8_t level)
{
    /* Master speaker output volume (0-255). Used by the remote's TWEAK mode. */
    M5.Speaker.setVolume(level);
}

uint8_t stackchan_get_volume(void)
{
    return M5.Speaker.getVolume();
}

void stackchan_display_sleep(bool sleep)
{
    /* SPI panel sleep-in/out (ILI9342 on CoreS3). Backlight is handled
     * separately via setBrightness(); do both for the biggest idle win. */
    if (sleep) {
        M5.Display.sleep();
    } else {
        M5.Display.wakeup();
    }
}

static void stackchan_status_leds(uint8_t r, uint8_t g, uint8_t b)
{
    auto &i2c = M5.In_I2C;

    /* Enable servo/VM power rail (pin 0 output high). */
    i2c.bitOn(PY32_ADDR, REG_GPIO_M_L, 1 << VM_EN_PIN, PY32_FREQ);
    i2c.bitOn(PY32_ADDR, REG_GPIO_PU_L, 1 << VM_EN_PIN, PY32_FREQ);
    i2c.bitOn(PY32_ADDR, REG_GPIO_O_L, 1 << VM_EN_PIN, PY32_FREQ);

    /* Enable the WS2812 data pin (pin 13: output, pull-up, push-pull). */
    i2c.bitOn(PY32_ADDR, REG_GPIO_M_H, 1 << (RGB_PIN - 8), PY32_FREQ);
    i2c.bitOn(PY32_ADDR, REG_GPIO_PU_H, 1 << (RGB_PIN - 8), PY32_FREQ);
    i2c.bitOff(PY32_ADDR, REG_GPIO_DRV_H, 1 << (RGB_PIN - 8), PY32_FREQ);

    /* LED count. */
    i2c.writeRegister8(PY32_ADDR, REG_LED_CFG, LED_COUNT & 0x3F, PY32_FREQ);

    /* Fill all LEDs (RGB888 -> RGB565, low byte first). */
    uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    uint8_t data[2] = { (uint8_t)(c & 0xFF), (uint8_t)((c >> 8) & 0xFF) };
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        i2c.writeRegister(PY32_ADDR, REG_LED_RAM + i * 2, data, 2, PY32_FREQ);
    }

    /* Latch buffered colors to the physical LEDs (set refresh bit 6). */
    i2c.bitOn(PY32_ADDR, REG_LED_CFG, 0x40, PY32_FREQ);
}

static bool stackchan_mount_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 4,
        .format_if_mount_failed = false,
    };
    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(err));
        return false;
    }
    size_t total = 0, used = 0;
    esp_spiffs_info("storage", &total, &used);
    ESP_LOGI(TAG, "SPIFFS mounted: %u/%u bytes used", (unsigned)used, (unsigned)total);
    return true;
}

static void avatar_video_task(void *arg)
{
    (void)arg;
    auto &lcd = M5.Display;
    const int x = (lcd.width() - AVATAR_FRAME_W) / 2;   /* center on 320-wide */
    const int y = (lcd.height() - AVATAR_FRAME_H) / 2;

    /* Reusable read buffer (frames are a few KB each). */
    const size_t buf_cap = 24 * 1024;
    uint8_t *buf = (uint8_t *)malloc(buf_cap);
    if (buf == NULL) {
        ESP_LOGE(TAG, "avatar buffer alloc failed");
        vTaskDelete(NULL);
        return;
    }

    /* ---- Anti-flicker: clip the dot's strip out of the avatar blit ------ *
     * The readiness dot used to flicker. drawJpg decodes top-to-bottom and
     * writes each row to the LCD as it goes, so the top-right corner (where
     * the dot lives) is overwritten with bare JPEG at the very START of each
     * ~80ms frame; the dot was only repainted at the END. So the dot showed
     * for a few ms and was bare the rest of the frame = a fast blink.
     *
     * Tried the recommended full-frame double-buffer sprite first: it IS
     * flicker-free but too heavy on this board. A PSRAM framebuffer decodes
     * at ~150 ms/frame (~6.5fps, ~half the ~12fps baseline; decoding pixels
     * into slow PSRAM dominates). An INTERNAL-RAM framebuffer is fast
     * (~109 ms) but stealing 150 KB of DMA-capable internal RAM starves the
     * WiFi/TLS stack and the device boot-loops. Both verified on device.
     *
     * Chosen fix (keeps the fast, overlapped direct-to-LCD decode AND is
     * genuinely flicker-free): clip the JPEG blit to EXCLUDE the thin
     * rightmost strip that contains the dot, so the avatar frame NEVER
     * overwrites the dot's pixels. The dot is drawn once into that strip and
     * simply persists, rock-solid, refreshed only for colour changes. This
     * is unlike the naive "redraw the dot after each frame" (which still
     * flickers because the frame keeps re-baring the corner first). The
     * avatar is a glowing icon centred on a black background, so the frozen
     * ~20px right strip is black in every frame -> the freeze is invisible.
     * SPI-only; adds ZERO I2C traffic (never wedges the shared bus/AXP2101). */
    const int clip_w = AVATAR_FRAME_W - DOT_STRIP_W; /* draw x=0..clip_w-1 */

    /* Clear the frozen strip to black once so it matches the avatar bg, then
     * lay the dot into it. From here the avatar blit never touches this strip. */
    lcd.fillRect(x + clip_w, y, DOT_STRIP_W, AVATAR_FRAME_H, TFT_BLACK);
    draw_status_dot(lcd);

    /* Frame-time instrumentation: average over ~one idle-loop worth of frames
     * to confirm we hold the target fps (no regression from anti-flicker). */
    int64_t loop_us = 0;
    int loop_frames = 0;

    for (;;) {
        if (!s_avatar_playing) {
            /* Paused: the LCD is owned by the thinking orb, the camera photo,
             * a text/goodnight screen, etc. Yield and re-check. */
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Pull the frame sequence for the CURRENT presentation state from the
         * active media provider. This is the state->presentation seam: the
         * flash/SPIFFS provider maps the state to a range of f0xx.jpg now; an
         * SD/MJPEG provider can drop in later (stk_media_set_provider) with no
         * change here. */
        const stk_media_provider_t *mp = stk_media_get_provider();
        stk_pres_state_t cur = stackchan_presentation_lcd_state();
        int count = mp->frame_count(mp, cur);
        int fps = mp->fps(mp, cur);
        if (count < 1) count = 1;
        if (fps < 1) fps = 1;
        TickType_t frame_ticks = pdMS_TO_TICKS(1000 / fps);
        if (frame_ticks < 1) frame_ticks = 1;

        for (int idx = 0; idx < count; idx++) {
            if (!s_avatar_playing) {
                break;
            }
            /* Re-evaluate the state each frame so a transition (e.g. speaking
             * starts or ends) switches sequence at the next frame boundary --
             * clean START and clean STOP, never a stuck frame, and it yields
             * the LCD instantly when a higher-priority owner (camera) pauses
             * us via s_avatar_playing. */
            if (stackchan_presentation_lcd_state() != cur) {
                break;
            }
            TickType_t t0 = xTaskGetTickCount();
            int64_t w0 = esp_timer_get_time();
            int n = mp->get_frame(mp, cur, idx, buf, buf_cap);
            if (n > 0) {
                /* Restrict the blit to everything LEFT of the dot strip. The
                 * decoder still decodes the whole JPEG but only writes the
                 * clipped columns, so the dot's pixels are never disturbed. */
                lcd.setClipRect(x, y, clip_w, AVATAR_FRAME_H);
                lcd.drawJpg(buf, (size_t)n, x, y);
                lcd.clearClipRect();
                /* Refresh the dot (cheap; picks up colour/state changes). It is
                 * outside the clip region so it is never overdrawn otherwise.
                 * EXCEPTION: skip it during THINKING. That is the delicate
                 * window where the reply audio triggers M5.Speaker.begin()
                 * (AW88298 codec config over the shared internal I2C bus);
                 * A/B testing showed adding the per-frame dot draw during
                 * THINKING correlated with persistent Speaker.begin failures +
                 * the AXP2101 false-shutdown wedge. The t-frame clip itself
                 * still blits (SPI only); only the dot is held. The dot resumes
                 * the instant THINKING ends. */
                if (cur != STK_PRES_THINKING) {
                    draw_status_dot(lcd);
                }
            }
            loop_us += esp_timer_get_time() - w0;
            if (++loop_frames >= AVATAR_FRAME_COUNT) {
                ESP_LOGI(TAG, "avatar render avg %.1f ms/frame over %d frames (%s)",
                         (double)loop_us / 1000.0 / loop_frames, loop_frames,
                         stk_pres_state_name(cur));
                loop_us = 0;
                loop_frames = 0;
            }
            /* Pace to the target frame period, but ALWAYS yield at least one
             * tick so the core1 idle task runs and feeds the task watchdog. */
            TickType_t elapsed = xTaskGetTickCount() - t0;
            if (elapsed < frame_ticks) {
                vTaskDelay(frame_ticks - elapsed);
            } else {
                vTaskDelay(1);
            }
        }
    }
}

static void stackchan_boot_splash(void)
{
    auto &lcd = M5.Display;
    lcd.setRotation(1);
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setTextDatum(middle_center);
    lcd.setTextSize(3);
    lcd.drawString("OpenClaw", lcd.width() / 2, lcd.height() / 2);
}

void stackchan_boot(void)
{
    auto cfg = M5.config();
    cfg.output_power = true;  /* enable external bus power to the robot base */
    M5.begin(cfg);

    ESP_LOGI(TAG, "M5 board init done (board=%d)", (int)M5.getBoard());

    stackchan_boot_splash();
    M5.Display.setBrightness(51);  /* TEMP dev: ~20% screen brightness (5% was below the backlight floor) */
    stackchan_status_leds(12, 2, 0);  /* faint amber = resting/idle */

    /* Start the looping avatar video if the frames are present. */
    if (stackchan_mount_spiffs()) {
        xTaskCreatePinnedToCore(avatar_video_task, "avatar_vid", 8192, NULL, 4, NULL, 1);
        ESP_LOGI(TAG, "avatar video task started");
    } else {
        ESP_LOGW(TAG, "no SPIFFS; leaving splash on screen");
    }

    ESP_LOGI(TAG, "boot bring-up complete");
}
