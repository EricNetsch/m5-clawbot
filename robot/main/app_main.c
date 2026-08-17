/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_openclaw_node_example_cmd.h"
#include "esp_openclaw_node_example_repl.h"
#include "esp_openclaw_node_example_repl_cmd.h"
#include "esp_openclaw_node_example_saved_session_reconnect.h"
#include "esp_openclaw_node.h"
#include "esp_openclaw_node_wifi.h"
#include "stackchan_boot.h"
#include "stackchan_body.h"
#include "esp_openclaw_node_stackchan_rgb_cmd.h"
#include "stackchan_servo.h"
#include "stackchan_joyc.h"
#include "stackchan_voice.h"
#include "stackchan_voice_net.h"
#include "stackchan_camera.h"
#include "stackchan_espnow_remote.h"
#include "stackchan_tailscale.h"
#include "stackchan_ready.h"
#include "stackchan_presentation.h"
#include "stackchan_idle.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"
#include <time.h>
#include <sys/time.h>

static const char *TAG = "app_main";
/* Operator leg: carries the voice loop (operator.read/write/talk scopes). */
static esp_openclaw_node_handle_t s_node;
/* Node leg: a second, independent WS client that carries the node.* device
 * control surface (servo/body/node commands) so the device stays a
 * controllable node while ALSO holding the operator/voice session. The
 * gateway supports one paired identity holding both roles at once, but each
 * WS connection declares exactly one role, so we run two boring clients.
 * Only created when talk is enabled (otherwise s_node itself is the node
 * leg). */
static esp_openclaw_node_handle_t s_node_ctrl;

/* Name the WS leg a given node handle belongs to ("operator" | "node" | "?").
 * Used by the speaker.playStream single-fire guard to attribute/telemeter
 * which leg delivered a reply-audio command invocation. s_node_ctrl is the
 * node-role leg (created only in dual-role/talk builds); s_node is the
 * operator leg (and, in single-leg builds, the node leg too). */
const char *stackchan_gateway_leg_name(esp_openclaw_node_handle_t node);
const char *stackchan_gateway_leg_name(esp_openclaw_node_handle_t node)
{
    if (node != NULL && node == s_node_ctrl) return "node";
    if (node != NULL && node == s_node) return "operator";
    return "?";
}

/* Route post-connect gateway broadcast events (talk.event transcripts, chat
 * replies) into the voice network transport. */
static void handle_gateway_event(
    esp_openclaw_node_handle_t node,
    const char *event,
    const char *payload_json,
    void *user_ctx)
{
    (void)node;
    (void)user_ctx;
    stackchan_voice_net_on_gateway_event(event, payload_json);
}
static esp_openclaw_node_example_saved_session_reconnect_t s_saved_session_reconnect;
/* Independent reconnect lifecycle for the node leg (its own retry loop, auth
 * flow, and liveness — kept fully separate from the operator leg). */
static esp_openclaw_node_example_saved_session_reconnect_t s_ctrl_reconnect;

/* Set true only after SNTP confirms a real wall-clock time. Gates WG TAI64N. */
static volatile bool s_clock_synced = false;

/* Background clock sync: waits for Wi-Fi, runs SNTP, blocks nothing else. */
static void time_sync_task(void *arg)
{
    (void)arg;
    /* Wait for Wi-Fi so the NTP query can leave the LAN. */
    for (int i = 0; i < 120 && !esp_openclaw_node_wifi_is_connected(); i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    /* Publish the wifi sub-state into the readiness tracker (this task is the
     * first place that blocks on Wi-Fi coming up). */
    stackchan_ready_set_wifi(esp_openclaw_node_wifi_is_connected());

    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    cfg.start = true;
    cfg.server_from_dhcp = true;      /* also accept an NTP server from DHCP */
    cfg.renew_servers_after_new_IP = true;
    if (esp_netif_sntp_init(&cfg) != ESP_OK) {
        ESP_LOGW(TAG, "SNTP init failed");
        vTaskDelete(NULL);
        return;
    }

    /* Wait for the first sync so downstream TLS sees a valid clock. */
    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(30000)) == ESP_OK) {
        time_t now = 0;
        time(&now);
        /* Gate WG handshake timestamps on this: only NOW is the wall clock
         * trustworthy. Before this, the RTC could read a stale/future date and
         * a future TAI64N would poison the WireGuard peer's replay state
         * permanently (it stores the greatest timestamp seen and drops anything
         * lower forever). See openclaw_wall_clock_ready(). */
        s_clock_synced = true;
        ESP_LOGI(TAG, "time synced: %s", asctime(gmtime(&now)));
    } else {
        ESP_LOGW(TAG, "SNTP first sync timed out (will keep trying in background)");
    }
    vTaskDelete(NULL);
}

static void start_time_sync(void)
{
    xTaskCreate(time_sync_task, "time_sync", 4096, NULL, 4, NULL);
}

/* Strong override of the weak default in wireguard-platform-esp32.c. The WG
 * TAI64N timestamp source uses real wall-clock time ONLY after this returns
 * true; before SNTP sync it falls back to a low uptime-based value that can
 * never exceed real time, so a bad pre-sync clock can't poison the peer's
 * replay protection. */
bool openclaw_wall_clock_ready(void)
{
    return s_clock_synced;
}

static void handle_node_event(
    esp_openclaw_node_handle_t node,
    esp_openclaw_node_event_t event,
    const void *event_data,
    void *user_ctx)
{
    (void)node;

    esp_openclaw_node_example_saved_session_reconnect_handle_event(
        (esp_openclaw_node_example_saved_session_reconnect_t *)user_ctx,
        event,
        event_data);

    /* Feed the system-readiness state. Derive which gateway leg this is from
     * the reconnect struct the leg was created with (operator carries voice;
     * ctrl is the node-role control leg). */
    stackchan_gateway_leg_t leg =
        (user_ctx == &s_ctrl_reconnect) ? STACKCHAN_LEG_NODE
                                        : STACKCHAN_LEG_OPERATOR;
    if (event == ESP_OPENCLAW_NODE_EVENT_CONNECTED) {
        stackchan_ready_set_gateway(leg, true);
    } else if (event == ESP_OPENCLAW_NODE_EVENT_DISCONNECTED ||
               event == ESP_OPENCLAW_NODE_EVENT_CONNECT_FAILED) {
        stackchan_ready_set_gateway(leg, false);
    }

    if (event == ESP_OPENCLAW_NODE_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "OpenClaw session connected");
    } else if (event == ESP_OPENCLAW_NODE_EVENT_CONNECT_FAILED) {
        const esp_openclaw_node_connect_failed_event_t *failed = event_data;
        ESP_LOGW(
            TAG,
            "OpenClaw connect failed: reason=%d local_err=%s gateway_detail=%s",
            failed != NULL ? failed->reason : -1,
            failed != NULL ? esp_err_to_name(failed->local_err) : "n/a",
            failed != NULL && failed->gateway_detail_code != NULL ? failed->gateway_detail_code : "");
    } else if (event == ESP_OPENCLAW_NODE_EVENT_DISCONNECTED) {
        const esp_openclaw_node_disconnected_event_t *disconnected = event_data;
        ESP_LOGW(
            TAG,
            "OpenClaw disconnected: reason=%d local_err=%s",
            disconnected != NULL ? disconnected->reason : -1,
            disconnected != NULL ? esp_err_to_name(disconnected->local_err) : "n/a");
    }
}

#if !SOC_WIFI_SUPPORTED
#error "examples/esp32-node is a Wi-Fi-specific example and requires a Wi-Fi-capable ESP32 target"
#endif

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Initialize the system-readiness tracker before anything can push a
     * sub-state into it (wifi/tailscale/gateway legs) or read it (the LCD dot
     * drawn by the avatar loop). */
    stackchan_ready_init();

    /* Bring up the StackChan body (power + display + status LEDs) first so
     * the device visibly shows it is powered on. */
    stackchan_boot();
    stackchan_servo_start();  /* servo bus up + idle look-around (power rail enabled in boot) */
    stackchan_joyc_start();   /* optional Mini JoyC (Port A) live head control */
    stackchan_voice_start();  /* push-to-talk voice loop (JoyC button toggles mic) */
    /* State-driven presentation: subscribe to stackchan_ready + voice sub-states
     * and drive the LCD frame source (via the swappable media provider), head
     * pose, and the coarse-state LED ring per state. Must come after ready_init,
     * boot (LCD/LED/SPIFFS), servo start, and voice start. */
    stackchan_presentation_start();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_openclaw_node_wifi_start());

    /* Start SNTP time sync. A correct wall clock is REQUIRED for TLS server
     * certificate validation (LetsEncrypt notBefore/notAfter checks) on the
     * wss:// gateway connection, and keeps WireGuard handshake timestamps
     * (TAI64N) monotonic so the peer's replay protection accepts them. Without
     * this the clock sits at 1970 and every TLS handshake is rejected as
     * "certificate is not yet valid". Non-blocking: waits for Wi-Fi + first
     * sync in a background task so boot isn't stalled. */
    start_time_sync();

    /* ESP-NOW joystick-remote receiver. Coexists with the station brought up
     * above; it waits for Wi-Fi internally, then esp_now_init()s on top of it.
     * Placed after wifi_start so esp_now has a started Wi-Fi to attach to. */
    stackchan_espnow_remote_start();

    /* Bring the device onto the tailnet if a key was stored over the console.
     * Runs after Wi-Fi start; it waits for Wi-Fi internally. Silent no-op when
     * unconfigured. Once up, `gateway token <ws://100.x.x.x:PORT> <token>`
     * reaches the gateway over Tailscale instead of the LAN. */
    stackchan_tailscale_start_if_configured();

    esp_openclaw_node_config_t node_config = {0};
    esp_openclaw_node_config_init_default(&node_config);
    /* Talk mode drives the voice loop entirely on operator.read/operator.write
     * scopes. The gateway's roleScopesAllow() rejects operator.* scopes unless
     * the connect frame's role is "operator" (a "node" role only accepts node.*
     * scopes), so the setup-code bootstrap handoff returns
     * AUTH_BOOTSTRAP_TOKEN_INVALID when we request operator scopes as a node.
     * Present role="operator" when talk is enabled so the handoff grants the
     * operator leg; PAIRING_SETUP_BOOTSTRAP_PROFILE allows both node+operator. */
    if (stackchan_voice_net_talk_enabled()) {
        node_config.role = "operator";
    }
    node_config.event_cb = handle_node_event;
    node_config.event_user_ctx = &s_saved_session_reconnect;
    node_config.gateway_event_cb = handle_gateway_event;
    /* Verify the gateway's Tailscale Serve TLS cert (LetsEncrypt) against the
     * bundled root CAs. The wss:// URI uses the real MagicDNS hostname (resolved
     * locally via the lwIP DNS hostlist populated from MicroLink peers), so the
     * cert common-name check passes. Full chain verification = no security loss. */
    node_config.use_cert_bundle = true;
    ESP_ERROR_CHECK(esp_openclaw_node_create(&node_config, &s_node));
    ESP_ERROR_CHECK(esp_openclaw_node_example_register_node_commands(s_node));
    ESP_ERROR_CHECK(esp_openclaw_node_stackchan_register_body_commands(s_node));
    ESP_ERROR_CHECK(esp_openclaw_node_stackchan_register_servo_commands(s_node));
    ESP_ERROR_CHECK(stackchan_camera_register_commands(s_node));
    /* Advertise the Talk capability + operator scopes BEFORE the reconnect
     * task can open a session. Needs a one-time node re-approval by Eric. */
    ESP_ERROR_CHECK(stackchan_voice_net_register(s_node));
    ESP_ERROR_CHECK(
        esp_openclaw_node_example_saved_session_reconnect_start(
            &s_saved_session_reconnect,
            s_node,
            "esp_openclaw_node_reconnect"));
    ESP_ERROR_CHECK(stackchan_tailscale_register_command());
    ESP_ERROR_CHECK(stackchan_voice_register_command());
    ESP_ERROR_CHECK(stackchan_camera_register_console());
    /* One-shot startup photo: snap once the first time we reach SYS_READY, so the
     * session opens with the bot noticing the room. Fires at most once per boot. */
    stackchan_camera_arm_boot_capture();
    ESP_ERROR_CHECK(stackchan_power_register_command());
    /* Single short press of the power key = graceful shutdown (recenter + off). */
    ESP_ERROR_CHECK(stackchan_power_button_monitor_start());
    ESP_ERROR_CHECK(stackchan_voice_net_start(s_node));

    /* Second leg: the node-role client. Only needed when talk is enabled, since
     * without talk the primary s_node above is already the node-role client.
     * Two independent WS clients ride the same Tailscale link to the same
     * host:port — just two handshakes + two reconnect loops. The node token was
     * persisted alongside the operator token during pairing (the operator
     * hello-ok handoff stores the peer role's deviceToken under the "node"
     * key), so this leg SAVED_SESSION-connects on its own token with no extra
     * pairing. If it reports "no saved session", re-pair once via the REPL
     * (`gateway setup-code ...`) with the bootstrap profile to mint both. */
    if (stackchan_voice_net_talk_enabled()) {
        esp_openclaw_node_config_t ctrl_config = {0};
        esp_openclaw_node_config_init_default(&ctrl_config);
        ctrl_config.role = "node"; /* explicit; DEFAULT_ROLE is also "node" */
        /* SHARED device identity: the node leg rides the SAME device_id as the
         * operator leg. jems approves a single device for BOTH roles
         * (roles=operator,node), so one approved identity carries both role
         * tokens. Do NOT set identity_nvs_namespace here -> both legs load the
         * default identity/seed -> one device_id -> one approval on jems that
         * grants operator+node. (Splitting identities created a second device
         * that had to be approved separately; the dual-role single device is
         * the cleaner answer.) */
        ctrl_config.event_cb = handle_node_event;
        ctrl_config.event_user_ctx = &s_ctrl_reconnect;
        /* node.event(voice.transcript / chat.subscribe) is node-role-only, so
         * voice_net emits those over THIS leg. Because chat.subscribe rides
         * this connection, the assistant's chat-reply broadcast comes back on
         * this leg too -> wire gateway_event_cb here so replies reach voice_net
         * (talk.event transcripts still arrive on the operator leg where the
         * talk.session lives; each event type lands on exactly one leg, so no
         * double-inject). */
        ctrl_config.gateway_event_cb = handle_gateway_event;
        ctrl_config.use_cert_bundle = true;
        ESP_ERROR_CHECK(esp_openclaw_node_create(&ctrl_config, &s_node_ctrl));
        ESP_ERROR_CHECK(esp_openclaw_node_example_register_node_commands(s_node_ctrl));
        ESP_ERROR_CHECK(esp_openclaw_node_stackchan_register_body_commands(s_node_ctrl));
        ESP_ERROR_CHECK(esp_openclaw_node_stackchan_register_servo_commands(s_node_ctrl));
        ESP_ERROR_CHECK(stackchan_camera_register_commands(s_node_ctrl));
        ESP_ERROR_CHECK(
            esp_openclaw_node_example_saved_session_reconnect_start(
                &s_ctrl_reconnect,
                s_node_ctrl,
                "esp_openclaw_ctrl_reconnect"));
        /* Route node-role-only node.event calls (voice.transcript /
         * chat.subscribe) over this leg instead of the operator leg. */
        stackchan_voice_net_set_node_leg(s_node_ctrl);
        /* This is a dual-role build: READY now requires BOTH gateway legs. */
        stackchan_ready_set_node_leg_required(true);
        ESP_LOGI(TAG, "node-role leg started (dual-role: operator + node)");
    }

    /* Bind the REPL `gateway` pairing command to whichever leg still needs a
     * token. The operator leg persists its own saved session, so once paired
     * it self-reconnects without the REPL. The node leg needs a DIRECT
     * role=node pairing (jems' operator hello-ok isn't handing back a node
     * deviceToken), so point the REPL at it when it exists. */
    ESP_ERROR_CHECK(esp_openclaw_node_example_repl_start(
        s_node_ctrl != NULL ? s_node_ctrl : s_node));

    /* Tiered idle/sleep coordinator (queue item #5): after 5 min of no activity
     * shed screen/render/servo/LED + enable WiFi modem power-save while keeping
     * both WS legs HOT for instant wake; after 30 min run the existing graceful
     * power-down. Started last so WiFi + all activity sources are up. The `idle`
     * REPL command (now/wake/tier2/status) gives a fast test path. */
    ESP_ERROR_CHECK(stackchan_idle_register_command());
    stackchan_idle_start();

    if (!esp_openclaw_node_wifi_is_connected()) {
        ESP_LOGI(TAG, "wifi not connected yet; provision it from the REPL or wait for Wi-Fi to reconnect");
        ESP_LOGI(TAG, "after Wi-Fi is up, use `gateway setup-code`, `gateway token`, `gateway password`, or `gateway no-auth` for a first connect");
        ESP_LOGI(TAG, "saved-session reconnect runs automatically after Wi-Fi returns when a saved session is present");
    }
}
