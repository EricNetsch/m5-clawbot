/*
 * StackChan Tailscale (MicroLink) integration.
 *
 * Brings the device onto the tailnet so the OpenClaw node can reach the
 * gateway over Tailscale instead of the local LAN. The auth key is stored in
 * NVS (set once over the console), so it survives reboots and never needs to
 * be flashed in.
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Persist the Tailscale auth key (tskey-auth-...) to NVS. Does not bring the
 * tunnel up on its own; call stackchan_tailscale_up() after. */
esp_err_t stackchan_tailscale_set_key(const char *auth_key);

/* Persist an optional device hostname on the tailnet (default "stackchan"). */
esp_err_t stackchan_tailscale_set_name(const char *device_name);

/* Erase the stored key/name and MicroLink's cached identity (factory reset). */
esp_err_t stackchan_tailscale_clear(void);

/* Bring the tunnel up using the NVS-stored key. Wi-Fi must be connected.
 * No-op (returns ESP_OK) if already up. Returns ESP_ERR_NOT_FOUND if no key
 * has been stored yet. */
esp_err_t stackchan_tailscale_up(void);

/* Tear the tunnel down. */
esp_err_t stackchan_tailscale_down(void);

/* Print current state (connection state, VPN IP, peer count) to stdout. */
void stackchan_tailscale_status(void);

/* Called at boot after Wi-Fi start: if a key is stored, bring the tunnel up
 * automatically. Silent no-op when unconfigured. */
void stackchan_tailscale_start_if_configured(void);

/* Register the `tailscale` console command. */
esp_err_t stackchan_tailscale_register_command(void);

#ifdef __cplusplus
}
#endif
