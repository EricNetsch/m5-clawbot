/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_openclaw_node.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bring up the Feetech SCSCL bus (UART1, 1 Mbaud, TX=6/RX=7), enable
 *        torque on the yaw+pitch servos, center the head, and start the subtle
 *        idle "look around" animation task.
 *
 * Servo power (PY32 io-expander pin 0 / VM_EN) must already be enabled — the
 * RGB init does this at boot.
 */
esp_err_t stackchan_servo_start(void);

/** @brief Enable/disable the idle look-around loop. */
void stackchan_servo_idle_set(bool enabled);

/**
 * @brief Move the head to an absolute pose in degrees relative to center.
 *        yaw: negative=left, positive=right. pitch: negative=down, positive=up.
 *        Values are clamped to safe mechanical limits in firmware.
 *        Pauses idle; call stackchan_servo_idle_set(true) to resume wandering.
 * @param speed 0-1000 (maps to move time; higher = faster).
 */
void stackchan_servo_move(int yaw_deg, int pitch_deg, int speed);

/** @brief Recenter the head (yaw 0, pitch 0). */
void stackchan_servo_home(void);

/**
 * @brief Last commanded head aim in degrees (best-effort; reflects the last
 *        commanded target from manual move / home / idle wander, not a servo
 *        readback). yaw: negative=left, positive=right. pitch: negative=down,
 *        positive=up. Either pointer may be NULL.
 */
void stackchan_servo_get_aim(int *yaw_deg, int *pitch_deg);

/**
 * @brief Shutdown pose: stop idle, glide the head back to dead center (0,0),
 *        then release servo torque so it is safe to power off / handle.
 */
void stackchan_servo_shutdown(void);

/**
 * @brief Idle relax: stop the idle animation and release torque on both servos
 *        so the head draws no holding current while the device is in light idle.
 *        The head goes limp where it is (intended -- it is sleeping). Reuses the
 *        same torque-release the shutdown path uses, minus the recenter/glide.
 *        Reverse with stackchan_servo_wake().
 */
void stackchan_servo_relax(void);

/**
 * @brief Wake from idle relax: re-enable torque on both servos and gently
 *        recenter so a coordinator pose apply can move the head again.
 */
void stackchan_servo_wake(void);

/**
 * @brief Play the cute startup pose sequence (look up, right, left, recenter
 *        looking up). Edit the BOOT_SEQUENCE table in stackchan_servo.c to
 *        change the choreography. Runs automatically at servo start; exposed
 *        here so it can be re-triggered on demand.
 */
void stackchan_servo_boot_sequence(void);

/**
 * @brief Register servo node commands (servo.move / servo.home / servo.idle).
 */
esp_err_t esp_openclaw_node_stackchan_register_servo_commands(esp_openclaw_node_handle_t node);

#ifdef __cplusplus
}
#endif
