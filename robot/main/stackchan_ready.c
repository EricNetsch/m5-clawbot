/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * StackChan system-readiness state -- see stackchan_ready.h.
 */
#include "stackchan_ready.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "stackchan_ready";

#define READY_MAX_SUBSCRIBERS 4

/* Sub-states, each pushed in from its real source. */
static bool s_wifi = false;
static bool s_tailscale = false;
static bool s_gw_operator = false;
static bool s_gw_node = false;

/* Whether an optional dependency gates READY (see header). */
static bool s_node_required = false;
static bool s_tailscale_required = false;

/* Latched: have we ever been fully ready? Distinguishes CONNECTING (never up)
 * from DEGRADED (was up, dropped). */
static bool s_ever_ready = false;

static stackchan_ready_state_t s_state = SYS_BOOTING;

static SemaphoreHandle_t s_lock = NULL;

typedef struct {
    stackchan_ready_change_cb_t cb;
    void *ctx;
} ready_sub_t;
static ready_sub_t s_subs[READY_MAX_SUBSCRIBERS];
static int s_sub_count = 0;

static inline void lock(void)
{
    if (s_lock != NULL) {
        xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    }
}
static inline void unlock(void)
{
    if (s_lock != NULL) {
        xSemaphoreGiveRecursive(s_lock);
    }
}

const char *stackchan_ready_state_name(stackchan_ready_state_t state)
{
    switch (state) {
        case SYS_BOOTING:    return "BOOTING";
        case SYS_CONNECTING: return "CONNECTING";
        case SYS_READY:      return "READY";
        case SYS_DEGRADED:   return "DEGRADED";
        default:             return "?";
    }
}

void stackchan_ready_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateRecursiveMutex();
    }
}

/* Compute whether every required sub-state is currently up. Caller holds lock. */
static bool compute_all_up(void)
{
    bool legs_up = s_gw_operator && (s_node_required ? s_gw_node : true);
    bool ts_up = s_tailscale_required ? s_tailscale : true;
    return s_wifi && ts_up && legs_up;
}

/* Recompute the coarse state from sub-states. Must be called with the lock
 * held; if the coarse state changed it fires subscribers AFTER dropping the
 * lock (so a callback can safely re-enter this module). */
static void recompute_locked(void)
{
    bool all_up = compute_all_up();
    stackchan_ready_state_t next;

    if (all_up) {
        next = SYS_READY;
        s_ever_ready = true;
    } else if (s_ever_ready) {
        next = SYS_DEGRADED;
    } else if (s_wifi || s_tailscale || s_gw_operator || s_gw_node) {
        next = SYS_CONNECTING;
    } else {
        next = SYS_BOOTING;
    }

    if (next == s_state) {
        return;
    }

    stackchan_ready_state_t prev = s_state;
    s_state = next;

    /* Snapshot subscribers, then fire outside the lock. */
    ready_sub_t subs[READY_MAX_SUBSCRIBERS];
    int n = s_sub_count;
    for (int i = 0; i < n; i++) {
        subs[i] = s_subs[i];
    }

    unlock();
    ESP_LOGI(TAG, "SYS -> %s (was %s) [wifi=%d ts=%d(req=%d) op=%d node=%d(req=%d)]",
             stackchan_ready_state_name(next), stackchan_ready_state_name(prev),
             s_wifi, s_tailscale, s_tailscale_required,
             s_gw_operator, s_gw_node, s_node_required);
    for (int i = 0; i < n; i++) {
        if (subs[i].cb != NULL) {
            subs[i].cb(next, subs[i].ctx);
        }
    }
    lock();
}

stackchan_ready_state_t stackchan_ready_get(void)
{
    lock();
    stackchan_ready_state_t s = s_state;
    unlock();
    return s;
}

bool stackchan_ready_is_ready(void)
{
    lock();
    bool r = compute_all_up();
    unlock();
    return r;
}

void stackchan_ready_set_wifi(bool up)
{
    lock();
    if (s_wifi != up) {
        s_wifi = up;
        recompute_locked();
    }
    unlock();
}

void stackchan_ready_set_tailscale(bool up)
{
    lock();
    if (s_tailscale != up) {
        s_tailscale = up;
        recompute_locked();
    }
    unlock();
}

void stackchan_ready_set_gateway(stackchan_gateway_leg_t leg, bool up)
{
    lock();
    bool changed = false;
    if (leg == STACKCHAN_LEG_OPERATOR) {
        if (s_gw_operator != up) { s_gw_operator = up; changed = true; }
    } else if (leg == STACKCHAN_LEG_NODE) {
        if (s_gw_node != up) { s_gw_node = up; changed = true; }
    }
    if (changed) {
        recompute_locked();
    }
    unlock();
}

void stackchan_ready_set_node_leg_required(bool required)
{
    lock();
    if (s_node_required != required) {
        s_node_required = required;
        recompute_locked();
    }
    unlock();
}

void stackchan_ready_set_tailscale_required(bool required)
{
    lock();
    if (s_tailscale_required != required) {
        s_tailscale_required = required;
        recompute_locked();
    }
    unlock();
}

esp_err_t stackchan_ready_on_change(stackchan_ready_change_cb_t cb, void *ctx)
{
    if (cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    lock();
    if (s_sub_count >= READY_MAX_SUBSCRIBERS) {
        unlock();
        return ESP_ERR_NO_MEM;
    }
    s_subs[s_sub_count].cb = cb;
    s_subs[s_sub_count].ctx = ctx;
    s_sub_count++;
    unlock();
    return ESP_OK;
}
