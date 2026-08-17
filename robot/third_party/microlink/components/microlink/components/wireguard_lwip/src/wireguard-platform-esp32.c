/**
 * @file wireguard-platform-esp32.c
 * @brief ESP32 platform implementation for wireguard-lwip
 */

#include "wireguard-platform.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs.h"
#include "lwip/sys.h"
#include <string.h>
#include <stdbool.h>
#include <sys/time.h>
#include <time.h>

/* Persistent monotonic TAI64N floor (seconds). WireGuard's responder keeps the
 * greatest handshake timestamp it has seen per peer and drops anything <= that.
 * Across reboots the RTC restarts unsynced, so a naive uptime/zero timestamp is
 * LOWER than a real wall-clock timestamp we sent in a previous session -> the
 * peer rejects every init and the tunnel deadlocks until the peer's WG engine is
 * reset (previously only fixed by restarting Tailscale on the gateway host).
 * We persist the greatest seconds value we've ever emitted in NVS and, when the
 * wall clock isn't trustworthy, emit floor+uptime so our timestamp ALWAYS keeps
 * increasing across reboots and is accepted without any gateway-side reset.
 * Writes are throttled to spare flash. */
#define WG_TAI_NVS_NS   "wgtai"
#define WG_TAI_NVS_KEY  "floor"
static uint64_t s_tai_floor_seconds = 0;   /* greatest emitted, persisted */
static uint64_t s_tai_last_seconds = 0;    /* strictly-increasing guard, this boot */
static bool     s_tai_loaded = false;

static void wg_tai_floor_load(void) {
    if (s_tai_loaded) return;
    s_tai_loaded = true;
    nvs_handle_t h;
    if (nvs_open(WG_TAI_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint64_t v = 0;
        if (nvs_get_u64(h, WG_TAI_NVS_KEY, &v) == ESP_OK) {
            s_tai_floor_seconds = v;
        }
        nvs_close(h);
    }
}

static void wg_tai_floor_persist(uint64_t seconds) {
    static uint64_t last_written = 0;
    /* Only write when it advances by >= 60s to limit flash wear. */
    if (seconds <= last_written + 60) return;
    nvs_handle_t h;
    if (nvs_open(WG_TAI_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        if (nvs_set_u64(h, WG_TAI_NVS_KEY, seconds) == ESP_OK) {
            nvs_commit(h);
            last_written = seconds;
        }
        nvs_close(h);
    }
}

/* Returns true once the app has a trustworthy (SNTP-synced) wall clock. The
 * application (app_main.c) provides a strong override; this weak default keeps
 * the component buildable and conservative (assume synced) if unused. */
__attribute__((weak)) bool openclaw_wall_clock_ready(void) { return true; }

/* ============================================================================
 * Time Functions
 * ========================================================================== */

uint32_t wireguard_sys_now() {
    // Use lwIP's built-in time function
    return sys_now();
}

void wireguard_tai64n_now(uint8_t *output) {
    // TAI64N format: 8 bytes seconds + 4 bytes nanoseconds.
    //
    // CRITICAL: this MUST be real wall-clock (Unix) time, and MUST increase on
    // every call across reboots. WireGuard's responder keeps the greatest
    // timestamp it has seen from each peer and silently drops any handshake
    // whose timestamp is <= that (replay protection). Using boot-relative time
    // (esp_timer_get_time) meant every reboot restarted near 1970, so after the
    // first successful handshake the peer (e.g. the gateway) rejected ALL
    // subsequent inits until its state reset. Wall-clock time from SNTP is
    // strictly increasing across reboots, so the responder always accepts us.
    struct timeval tv = {0};
    gettimeofday(&tv, NULL);
    uint64_t seconds = (uint64_t)tv.tv_sec;
    uint32_t nanoseconds = (uint32_t)tv.tv_usec * 1000u;

    // Trust the wall clock ONLY after SNTP has confirmed a real sync. Before
    // that the RTC can read a stale OR FUTURE date. Instead of a low uptime
    // value (which is LOWER than a real timestamp we emitted in a prior session
    // and gets replay-rejected by the peer forever), emit persisted_floor +
    // uptime. That is strictly increasing across reboots and always >= the
    // greatest timestamp the peer has stored from us, so the handshake is
    // accepted with no gateway-side WireGuard reset. Once SNTP syncs, real time
    // exceeds the fallback and we persist the new high-water floor.
    wg_tai_floor_load();
    bool synced = openclaw_wall_clock_ready() && seconds >= 1600000000ULL;
    if (synced) {
        if (seconds > s_tai_floor_seconds) {
            s_tai_floor_seconds = seconds;
        }
        wg_tai_floor_persist(s_tai_floor_seconds);
    } else {
        uint64_t up = esp_timer_get_time() / 1000000ULL;
        seconds = s_tai_floor_seconds + up + 1ULL;
        nanoseconds = 0;
        static uint64_t last_warn_s = 0;
        if (seconds - last_warn_s >= 10) {
            ESP_LOGW("wg_tai64n", "clock not synced; TAI64N=floor(%llu)+uptime -> monotonic, replay-safe",
                     (unsigned long long)s_tai_floor_seconds);
            last_warn_s = seconds;
        }
    }
    // Strictly-increasing guard within this boot (covers rapid successive calls
    // and the synced/unsynced boundary).
    if (seconds <= s_tai_last_seconds) {
        seconds = s_tai_last_seconds + 1ULL;
    }
    s_tai_last_seconds = seconds;

    // TAI64 starts at 1970-01-01 00:00:10 TAI (Unix epoch + 10 seconds)
    // Add TAI offset: 2^62 + Unix time
    seconds += 0x400000000000000AULL;

    // Write in big-endian format
    output[0] = (seconds >> 56) & 0xFF;
    output[1] = (seconds >> 48) & 0xFF;
    output[2] = (seconds >> 40) & 0xFF;
    output[3] = (seconds >> 32) & 0xFF;
    output[4] = (seconds >> 24) & 0xFF;
    output[5] = (seconds >> 16) & 0xFF;
    output[6] = (seconds >> 8) & 0xFF;
    output[7] = seconds & 0xFF;

    output[8] = (nanoseconds >> 24) & 0xFF;
    output[9] = (nanoseconds >> 16) & 0xFF;
    output[10] = (nanoseconds >> 8) & 0xFF;
    output[11] = nanoseconds & 0xFF;
}

/* ============================================================================
 * Random Number Generation
 * ========================================================================== */

void wireguard_random_bytes(void *bytes, size_t size) {
    // Use ESP32 hardware RNG
    esp_fill_random(bytes, size);
}

/* ============================================================================
 * Load Management
 * ========================================================================== */

bool wireguard_is_under_load() {
    // For now, always return false (not under load)
    // Could be enhanced to check:
    // - Free heap memory
    // - CPU usage
    // - Number of active connections
    return false;
}
