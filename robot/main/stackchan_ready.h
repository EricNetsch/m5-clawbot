/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * StackChan system-readiness state.
 *
 * A single, reusable place that tracks whether the device is fully up and
 * ready to accept OpenClaw commands:
 *   WIFI + TAILSCALE (if configured) + the required gateway leg(s).
 *
 * Any subsystem can query the coarse state, ask "am I ready?", or subscribe to
 * ready-state transitions via a callback. Sub-states are pushed in from their
 * real sources (wifi start, tailscale link, gateway CONNECTED/DISCONNECTED
 * events) -- see the setters below, wired from app_main.
 *
 * Thread-safe: setters/getters may be called from event, timer, and task
 * contexts. Registered change callbacks are invoked OUTSIDE the internal lock
 * so a callback may safely call back into this module.
 *
 * NOTE: this module never touches hardware. The LCD "green dot" that reflects
 * this state is drawn by the avatar loop (SPI display, off the fragile shared
 * I2C bus) -- this header exposes only the state.
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Coarse, human-meaningful readiness. */
typedef enum {
    SYS_BOOTING = 0,   /* just powered on; nothing up yet */
    SYS_CONNECTING,    /* making progress (wifi/tailscale/a leg up) but not ready */
    SYS_READY,         /* wifi + tailscale(if req) + required gateway legs all up */
    SYS_DEGRADED,      /* was READY, then a required sub-state dropped */
} stackchan_ready_state_t;

/* The two independent gateway WS connections. Operator carries the voice/talk
 * session; node carries the device control surface. */
typedef enum {
    STACKCHAN_LEG_OPERATOR = 0,
    STACKCHAN_LEG_NODE,
} stackchan_gateway_leg_t;

/* Invoked (outside the lock) whenever stackchan_ready_get() changes value. */
typedef void (*stackchan_ready_change_cb_t)(stackchan_ready_state_t state, void *ctx);

/* Initialize the module (creates the internal lock). Idempotent; call once,
 * early in app_main, before any setter fires. */
void stackchan_ready_init(void);

/* Current coarse readiness. Safe before init (returns SYS_BOOTING). */
stackchan_ready_state_t stackchan_ready_get(void);

/* True iff wifi + tailscale(if required) + required gateway legs are all up. */
bool stackchan_ready_is_ready(void);

/* Sub-state setters (called from their real sources). */
void stackchan_ready_set_wifi(bool up);
void stackchan_ready_set_tailscale(bool up);
void stackchan_ready_set_gateway(stackchan_gateway_leg_t leg, bool up);

/* Declare whether each optional dependency gates READY.
 *  - node leg: only required in dual-role (talk-enabled) builds.
 *  - tailscale: only required when a tailnet key is configured; a LAN-only
 *    build reaches READY without it.
 * Both default to NOT required so a minimal single-leg / LAN build can still
 * become READY. */
void stackchan_ready_set_node_leg_required(bool required);
void stackchan_ready_set_tailscale_required(bool required);

/* Subscribe to ready-state transitions. Up to a small fixed number of
 * subscribers. Returns ESP_ERR_NO_MEM if the table is full. */
esp_err_t stackchan_ready_on_change(stackchan_ready_change_cb_t cb, void *ctx);

/* Human-readable name for a state (for logs). */
const char *stackchan_ready_state_name(stackchan_ready_state_t state);

#ifdef __cplusplus
}
#endif
