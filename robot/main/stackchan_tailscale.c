/*
 * StackChan Tailscale (MicroLink) integration — see stackchan_tailscale.h.
 */
#include "stackchan_tailscale.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_console.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "microlink.h"
#include "esp_openclaw_node_wifi.h"
#include "stackchan_ready.h"

#include "lwip/dns.h"
#include "lwip/ip_addr.h"

static const char *TAG = "stackchan_tailscale";

#define TS_NVS_NAMESPACE   "stackchan_ts"
#define TS_NVS_KEY_AUTH    "auth_key"
#define TS_NVS_KEY_NAME    "dev_name"

#define TS_AUTH_KEY_MAX    96
#define TS_DEV_NAME_MAX    48
#define TS_DEFAULT_NAME    "stackchan"

#define TS_WIFI_WAIT_TICKS pdMS_TO_TICKS(30000)

/* Held for the lifetime of the tunnel. MicroLink copies the config struct but
 * keeps the auth_key/device_name pointers, so these buffers must persist. */
static microlink_t *s_ml = NULL;
static char s_auth_key[TS_AUTH_KEY_MAX];
static char s_dev_name[TS_DEV_NAME_MAX];

static void ts_dns_sync_task(void *arg);

/* ------------------------------------------------------------------ */
/* NVS helpers                                                         */
/* ------------------------------------------------------------------ */

static esp_err_t ts_nvs_set_str(const char *key, const char *value)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(TS_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(h, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

static esp_err_t ts_nvs_get_str(const char *key, char *out, size_t out_len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(TS_NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return err;
    }
    size_t len = out_len;
    err = nvs_get_str(h, key, out, &len);
    nvs_close(h);
    return err;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t stackchan_tailscale_set_key(const char *auth_key)
{
    if (auth_key == NULL || auth_key[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    if (strlen(auth_key) >= TS_AUTH_KEY_MAX) {
        ESP_LOGE(TAG, "auth key too long (max %d)", TS_AUTH_KEY_MAX - 1);
        return ESP_ERR_INVALID_SIZE;
    }
    return ts_nvs_set_str(TS_NVS_KEY_AUTH, auth_key);
}

esp_err_t stackchan_tailscale_set_name(const char *device_name)
{
    if (device_name == NULL || device_name[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    if (strlen(device_name) >= TS_DEV_NAME_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }
    return ts_nvs_set_str(TS_NVS_KEY_NAME, device_name);
}

esp_err_t stackchan_tailscale_clear(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(TS_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    /* Also wipe MicroLink's persisted machine/WG keys + peer cache so a fresh
     * key registers a clean device. Safe to call before init. */
    microlink_factory_reset();
    return ESP_OK;
}

esp_err_t stackchan_tailscale_up(void)
{
    if (s_ml != NULL) {
        ESP_LOGI(TAG, "tailscale already up");
        return ESP_OK;
    }

    esp_err_t err = ts_nvs_get_str(TS_NVS_KEY_AUTH, s_auth_key, sizeof(s_auth_key));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no stored tailscale key (%s)", esp_err_to_name(err));
        return ESP_ERR_NOT_FOUND;
    }

    if (ts_nvs_get_str(TS_NVS_KEY_NAME, s_dev_name, sizeof(s_dev_name)) != ESP_OK) {
        strncpy(s_dev_name, TS_DEFAULT_NAME, sizeof(s_dev_name) - 1);
        s_dev_name[sizeof(s_dev_name) - 1] = '\0';
    }

    if (!esp_openclaw_node_wifi_is_connected()) {
        ESP_LOGI(TAG, "waiting for Wi-Fi before tailscale up");
        if (!esp_openclaw_node_wifi_wait_for_connection(TS_WIFI_WAIT_TICKS)) {
            ESP_LOGW(TAG, "Wi-Fi not connected; tailscale not started");
            return ESP_ERR_TIMEOUT;
        }
    }

    microlink_config_t config = {
        .auth_key = s_auth_key,
        .device_name = s_dev_name,
        .enable_derp = true,
        .enable_stun = true,
        .enable_disco = true,
        .max_peers = 8,
    };

    s_ml = microlink_init(&config);
    if (s_ml == NULL) {
        ESP_LOGE(TAG, "microlink_init failed");
        return ESP_FAIL;
    }

    err = microlink_start(s_ml);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "microlink_start failed: %s", esp_err_to_name(err));
        microlink_destroy(s_ml);
        s_ml = NULL;
        return err;
    }

    /* A tailnet key is configured, so the tunnel now gates system readiness.
     * The ts_dns_sync_task loop below keeps the live link sub-state fresh. */
    stackchan_ready_set_tailscale_required(true);

    ESP_LOGI(TAG, "tailscale starting as '%s' (connects asynchronously)", s_dev_name);

    /* Publish tailnet MagicDNS names into the lwIP local DNS hostlist so the
     * gateway wss:// URI can use the real hostname (needed for TLS SNI + cert
     * common-name), while packets still route to the peer's tunnel IP. */
    xTaskCreate(ts_dns_sync_task, "ts_dns_sync", 3072, NULL, 4, NULL);

    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* MagicDNS -> lwIP local DNS hostlist bridge                          */
/* ------------------------------------------------------------------ */

static void ts_dns_sync_task(void *arg)
{
    (void)arg;
    while (s_ml != NULL) {
        /* Publish live tunnel link state into the readiness tracker so a
         * dropped tailnet link surfaces as DEGRADED and recovery flips back. */
        stackchan_ready_set_tailscale(microlink_get_state(s_ml) == ML_STATE_CONNECTED);

        int count = microlink_get_peer_count(s_ml);
        for (int i = 0; i < count; i++) {
            microlink_peer_info_t info;
            if (microlink_get_peer_info(s_ml, i, &info) != ESP_OK) {
                continue;
            }
            if (info.hostname[0] == '\0' || info.vpn_ip == 0) {
                continue;
            }
            /* vpn_ip is host byte order with the first octet in the high byte. */
            uint32_t v = info.vpn_ip;
            ip_addr_t ipaddr;
            IP_ADDR4(&ipaddr,
                     (v >> 24) & 0xFF, (v >> 16) & 0xFF,
                     (v >> 8) & 0xFF, v & 0xFF);
            /* Refresh: drop any stale entry, then add the current mapping. */
            dns_local_removehost(info.hostname, NULL);
            dns_local_addhost(info.hostname, &ipaddr);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    vTaskDelete(NULL);
}

esp_err_t stackchan_tailscale_down(void)
{
    if (s_ml == NULL) {
        return ESP_OK;
    }
    microlink_stop(s_ml);
    microlink_destroy(s_ml);
    s_ml = NULL;
    stackchan_ready_set_tailscale(false);
    ESP_LOGI(TAG, "tailscale stopped");
    return ESP_OK;
}

void stackchan_tailscale_status(void)
{
    char stored[TS_AUTH_KEY_MAX];
    bool has_key = (ts_nvs_get_str(TS_NVS_KEY_AUTH, stored, sizeof(stored)) == ESP_OK);
    printf("tailscale key stored: %s\n", has_key ? "yes" : "no");

    if (s_ml == NULL) {
        printf("tailscale tunnel: down\n");
        return;
    }

    microlink_state_t st = microlink_get_state(s_ml);
    const char *st_str = "unknown";
    switch (st) {
        case ML_STATE_IDLE:         st_str = "idle";         break;
        case ML_STATE_WIFI_WAIT:    st_str = "wifi-wait";    break;
        case ML_STATE_CONNECTING:   st_str = "connecting";   break;
        case ML_STATE_REGISTERING:  st_str = "registering";  break;
        case ML_STATE_CONNECTED:    st_str = "connected";    break;
        case ML_STATE_RECONNECTING: st_str = "reconnecting"; break;
        case ML_STATE_ERROR:        st_str = "error";        break;
    }
    printf("tailscale tunnel: %s\n", st_str);

    uint32_t ip = microlink_get_vpn_ip(s_ml);
    if (ip != 0) {
        char ipbuf[16];
        microlink_ip_to_str(ip, ipbuf);
        printf("tailscale ip: %s\n", ipbuf);
    }
    printf("tailscale peers: %d\n", microlink_get_peer_count(s_ml));
}

void stackchan_tailscale_start_if_configured(void)
{
    char stored[TS_AUTH_KEY_MAX];
    if (ts_nvs_get_str(TS_NVS_KEY_AUTH, stored, sizeof(stored)) != ESP_OK) {
        ESP_LOGI(TAG, "no tailscale key stored; skipping (set with `tailscale up <key>`)");
        return;
    }
    esp_err_t err = stackchan_tailscale_up();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "auto tailscale up failed: %s", esp_err_to_name(err));
    }
}

/* ------------------------------------------------------------------ */
/* Console command                                                     */
/* ------------------------------------------------------------------ */

static int handle_tailscale_command(int argc, char **argv)
{
    /* tailscale up [<key>]  — store key (if given) and bring tunnel up */
    if (argc >= 2 && strcmp(argv[1], "up") == 0) {
        if (argc >= 3) {
            esp_err_t err = stackchan_tailscale_set_key(argv[2]);
            if (err != ESP_OK) {
                printf("failed to save key: %s\n", esp_err_to_name(err));
                return 1;
            }
            printf("tailscale key saved\n");
            if (argc >= 4) {
                if (stackchan_tailscale_set_name(argv[3]) == ESP_OK) {
                    printf("tailscale device name saved: %s\n", argv[3]);
                }
            }
        }
        esp_err_t err = stackchan_tailscale_up();
        if (err == ESP_ERR_NOT_FOUND) {
            printf("no key stored; use `tailscale up <tskey-auth-...>`\n");
            return 1;
        }
        if (err != ESP_OK) {
            printf("tailscale up failed: %s\n", esp_err_to_name(err));
            return 1;
        }
        printf("tailscale up requested (connects asynchronously; run `tailscale status`)\n");
        return 0;
    }

    if (argc == 2 && strcmp(argv[1], "down") == 0) {
        stackchan_tailscale_down();
        printf("tailscale down\n");
        return 0;
    }

    if (argc == 2 && strcmp(argv[1], "status") == 0) {
        stackchan_tailscale_status();
        return 0;
    }

    if (argc == 3 && strcmp(argv[1], "name") == 0) {
        esp_err_t err = stackchan_tailscale_set_name(argv[2]);
        if (err != ESP_OK) {
            printf("failed to save name: %s\n", esp_err_to_name(err));
            return 1;
        }
        printf("tailscale device name saved: %s (restart or re-up to apply)\n", argv[2]);
        return 0;
    }

    if (argc == 2 && strcmp(argv[1], "clear") == 0) {
        stackchan_tailscale_clear();
        printf("tailscale key/identity cleared\n");
        return 0;
    }

    printf("usage: tailscale up [<tskey-auth-...>] [<device-name>]\n");
    printf("       tailscale down\n");
    printf("       tailscale status\n");
    printf("       tailscale name <device-name>\n");
    printf("       tailscale clear\n");
    return 1;
}

esp_err_t stackchan_tailscale_register_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "tailscale",
        .help = "Bring the device onto the tailnet (MicroLink). Key persists in NVS.",
        .hint = "up [<key>] [<name>] | down | status | name <name> | clear",
        .func = handle_tailscale_command,
    };
    return esp_console_cmd_register(&cmd);
}
