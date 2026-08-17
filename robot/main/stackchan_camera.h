/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * StackChan CoreS3 camera: snap a frame from the onboard GC0308, show it on the
 * LCD for a few seconds, and hand the JPEG to OpenClaw (node.event agent.request
 * with an image attachment) so the gateway model can analyze it.
 *
 * Shared-bus discipline: the GC0308 SCCB control lines are on the internal I2C
 * bus that M5.In_I2C owns. All sensor register I/O is routed through M5.In_I2C
 * (see components/esp32-camera/driver/sccb.c + stk_cam_sccb_* here); the camera
 * NEVER installs a second i2c driver. The DVP pixel path is a separate parallel
 * bus. Frame buffers live in PSRAM so the speaker's scarce internal DMA is
 * untouched, and the camera is deinited right after each capture so it holds no
 * DMA/PSRAM between shots.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "esp_openclaw_node.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Kick off a snap -> display(~3s) -> upload-to-OpenClaw cycle on a
 *        one-shot worker task. Returns immediately. If a capture is already in
 *        flight, this is a no-op (single-flight guard) and returns false.
 *
 * @param prompt  Optional instruction sent alongside the image to the model
 *                (e.g. "What do you see?"). NULL/empty -> a default describe
 *                prompt. Copied internally.
 * @return true if a capture was started, false if one was already running.
 */
bool stackchan_camera_capture_async(const char *prompt);

/**
 * @brief Register the `camera.capture` node command on a gateway leg. Call once
 *        per WS leg (same both-legs pattern as the body/servo commands). A
 *        shared single-flight guard makes a double-delivery across legs safe.
 */
esp_err_t stackchan_camera_register_commands(esp_openclaw_node_handle_t node);

/** @brief Register the `camera` serial console command (snap/test). */
esp_err_t stackchan_camera_register_console(void);

/**
 * @brief Arm a ONE-SHOT camera capture that fires the first time the device
 *        reaches SYS_READY after boot (a short settle delay lets the post-
 *        connect DMA storm pass). Fires at most once per boot; safe to call
 *        multiple times. Subscribes to stackchan_ready internally, so call
 *        after stackchan_ready_init().
 */
void stackchan_camera_arm_boot_capture(void);

#ifdef __cplusplus
}
#endif
