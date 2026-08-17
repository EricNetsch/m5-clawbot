/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal native driver for the M5 StackChan head servos (Feetech SCSCL bus
 * servos) plus a subtle idle "look around" animation, ported from the M5
 * StackChan-BSP without the Arduino dependency.
 *
 * Bus: UART1, 1 Mbaud, TX=GPIO6, RX=GPIO7 (half-duplex, replies ignored).
 * Protocol frame: FF FF ID LEN INST MEMADDR [DATA..] ~CHECKSUM
 *   CHECKSUM = ~(ID + LEN + INST + MEMADDR + sum(DATA))
 * SCSCL is big-endian ("End=1"): 16-bit values are sent high byte first.
 */

#include "stackchan_servo.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "esp_openclaw_node_example_json.h"

static const char *TAG = "stackchan_servo";

/* --- Bus / servo config (from StackChan-BSP M5StackChan.cpp) --- */
#define SERVO_UART        UART_NUM_1
#define SERVO_BAUD        1000000
#define SERVO_TX_PIN      6
#define SERVO_RX_PIN      7

#define INST_WRITE        0x03
#define REG_TORQUE_ENABLE 40
#define REG_GOAL_POS      42   /* GOAL_POSITION_L; writes pos(2),time(2),speed(2) */

#define YAW_ID            1
#define PITCH_ID          2

/* Raw step = 0.3125 deg  ->  3.2 raw steps per degree. */
#define STEPS_PER_DEG_X10 32   /* steps per degree * 10 */

/* Center raw positions (BSP defaults). rawPosLimit is 0..1000. */
#define YAW_REST          460
#define PITCH_REST        640

/* Conservative safe travel window (well inside BSP mechanical limits). */
#define YAW_MIN           60    /* ~ -125 deg (factory full range) */
#define YAW_MAX           860   /* ~ +125 deg */
#define PITCH_MIN         620   /* mechanical floor (~ -6 deg from rest) */
#define PITCH_MAX         900   /* ~ +81 deg up (factory full range) */

static bool s_bus_ready = false;
/* TEMP dev switch: 1 = idle look-around fully disabled (head holds still).
 * Flip back to 0 to restore the ambient animation. */
#define DEV_DISABLE_IDLE 1

/* ===================================================================== *
 *  CUTE BOOT SEQUENCE  --  EDIT ME
 * ===================================================================== *
 * The head plays this pose list once at startup, then settles into idle.
 * Angles are degrees relative to center:
 *   yaw:   negative = LEFT,  positive = RIGHT
 *   pitch: negative = DOWN,  positive = UP
 * move_ms = how long to ease INTO the pose (bigger = slower/smoother).
 * hold_ms = how long to sit on the pose before the next one.
 * Add/remove/re-order rows freely; the sequence auto-sizes.
 * Set BOOT_SEQUENCE_ENABLE to 0 to skip the whole thing.
 */
#define BOOT_SEQUENCE_ENABLE 1

typedef struct {
    int      yaw_deg;   /* -left / +right */
    int      pitch_deg; /* -down / +up    */
    uint16_t move_ms;   /* ease-in time   */
    uint16_t hold_ms;   /* dwell on pose  */
} servo_boot_pose_t;

static const servo_boot_pose_t BOOT_SEQUENCE[] = {
    /* yaw, pitch, move_ms, hold_ms */
    {   0,   20,   450,   300 },  /* 1. look up a bit -- "huh?" */
    {  32,   16,   420,   260 },  /* 2. glance RIGHT           */
    { -32,   16,   620,   260 },  /* 3. sweep LEFT             */
    {   0,   24,   480,   250 },  /* 4. recenter + chin up -- ready for attention */
};

static volatile bool s_idle_enabled = !DEV_DISABLE_IDLE;
static TaskHandle_t s_idle_task = NULL;

/* Last commanded head aim in degrees (yaw: -left/+right, pitch: -down/+up),
 * updated by every path that moves the head (manual move, home, idle wander).
 * Best-effort: reflects the last *commanded* target, not a servo readback. */
static volatile int s_aim_yaw_deg = 0;
static volatile int s_aim_pitch_deg = 0;

static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Send an SCS WRITE packet (reply, if any, is flushed and ignored). */
static void scs_write(uint8_t id, uint8_t memaddr, const uint8_t *data, uint8_t len)
{
    uint8_t buf[16];
    uint8_t msglen = (uint8_t)(len + 3);  /* memaddr + data + 2 */
    uint8_t n = 0;
    buf[n++] = 0xFF;
    buf[n++] = 0xFF;
    buf[n++] = id;
    buf[n++] = msglen;
    buf[n++] = INST_WRITE;
    buf[n++] = memaddr;
    uint8_t checksum = id + msglen + INST_WRITE + memaddr;
    for (uint8_t i = 0; i < len; i++) {
        buf[n++] = data[i];
        checksum += data[i];
    }
    buf[n++] = (uint8_t)(~checksum);
    uart_write_bytes(SERVO_UART, (const char *)buf, n);
    uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(20));
    uart_flush_input(SERVO_UART);  /* drop the servo status reply */
}

static void scs_enable_torque(uint8_t id, bool en)
{
    uint8_t d = en ? 1 : 0;
    scs_write(id, REG_TORQUE_ENABLE, &d, 1);
}

/* Move one servo to a raw position (0..1000), easing over move_ms. */
static void scs_write_pos(uint8_t id, uint16_t pos, uint16_t move_ms)
{
    uint8_t d[6];
    d[0] = (uint8_t)(pos >> 8);      d[1] = (uint8_t)(pos & 0xFF);       /* position */
    d[2] = (uint8_t)(move_ms >> 8);  d[3] = (uint8_t)(move_ms & 0xFF);   /* time */
    d[4] = 0;                        d[5] = 0;                            /* speed=0 */
    scs_write(id, REG_GOAL_POS, d, 6);
}

static uint16_t yaw_deg_to_raw(int deg)
{
    int raw = YAW_REST + (deg * STEPS_PER_DEG_X10) / 10;
    return (uint16_t)clamp_int(raw, YAW_MIN, YAW_MAX);
}

static uint16_t pitch_deg_to_raw(int deg)
{
    int raw = PITCH_REST + (deg * STEPS_PER_DEG_X10) / 10;
    return (uint16_t)clamp_int(raw, PITCH_MIN, PITCH_MAX);
}

static uint32_t rnd(uint32_t lo, uint32_t hi)
{
    if (hi <= lo) return lo;
    return lo + (esp_random() % (hi - lo + 1));
}

/* Subtle idle: small, slow, organic wandering around center with occasional
 * pauses and returns to center. Stays far inside mechanical limits. */
static void idle_task(void *arg)
{
    (void)arg;
    int moves_since_center = 0;

    for (;;) {
        if (!s_idle_enabled) {
            vTaskDelay(pdMS_TO_TICKS(150));
            continue;
        }

        int yaw_deg, pitch_deg;
        if (moves_since_center >= (int)rnd(3, 6)) {
            /* Drift back toward center to keep the wander bounded. */
            yaw_deg = (int)rnd(0, 6) - 3;
            pitch_deg = (int)rnd(0, 6);
            moves_since_center = 0;
        } else {
            /* Small look-around: yaw +/-12 deg, pitch -4..+14 deg. */
            yaw_deg = (int)rnd(0, 24) - 12;
            pitch_deg = (int)rnd(0, 18) - 4;
            moves_since_center++;
        }
        uint16_t yaw = yaw_deg_to_raw(yaw_deg);
        uint16_t pitch = pitch_deg_to_raw(pitch_deg);
        s_aim_yaw_deg = yaw_deg;
        s_aim_pitch_deg = pitch_deg;

        uint16_t move_ms = (uint16_t)rnd(700, 1400);  /* gentle */
        scs_write_pos(YAW_ID, yaw, move_ms);
        vTaskDelay(pdMS_TO_TICKS(30));
        scs_write_pos(PITCH_ID, pitch, move_ms);

        /* Dwell: hold the pose a beat before the next drift. */
        uint32_t dwell = rnd(900, 2600);
        uint32_t waited = 0;
        while (waited < dwell) {
            if (!s_idle_enabled) break;
            vTaskDelay(pdMS_TO_TICKS(100));
            waited += 100;
        }
    }
}

/* Play the cute startup pose list once. Easy to edit via BOOT_SEQUENCE above. */
static void play_boot_sequence(void)
{
#if BOOT_SEQUENCE_ENABLE
    const size_t n = sizeof(BOOT_SEQUENCE) / sizeof(BOOT_SEQUENCE[0]);
    for (size_t i = 0; i < n; i++) {
        const servo_boot_pose_t *p = &BOOT_SEQUENCE[i];
        scs_write_pos(YAW_ID, yaw_deg_to_raw(p->yaw_deg), p->move_ms);
        vTaskDelay(pdMS_TO_TICKS(30));
        scs_write_pos(PITCH_ID, pitch_deg_to_raw(p->pitch_deg), p->move_ms);
        vTaskDelay(pdMS_TO_TICKS(p->move_ms + p->hold_ms));
    }
#endif
}

void stackchan_servo_boot_sequence(void)
{
    if (!s_bus_ready) return;
    s_idle_enabled = false;
    play_boot_sequence();
}

esp_err_t stackchan_servo_start(void)
{
    if (s_bus_ready) {
        return ESP_OK;
    }

    uart_config_t cfg = {
        .baud_rate = SERVO_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(SERVO_UART, 256, 256, 0, NULL, 0), TAG, "uart install");
    ESP_RETURN_ON_ERROR(uart_param_config(SERVO_UART, &cfg), TAG, "uart cfg");
    ESP_RETURN_ON_ERROR(
        uart_set_pin(SERVO_UART, SERVO_TX_PIN, SERVO_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
        TAG, "uart pins");

    s_bus_ready = true;

    /* Enable torque and center the head gently. */
    scs_enable_torque(YAW_ID, true);
    scs_enable_torque(PITCH_ID, true);
    scs_write_pos(YAW_ID, YAW_REST, 800);
    scs_write_pos(PITCH_ID, PITCH_REST, 800);
    vTaskDelay(pdMS_TO_TICKS(900));

    /* Cute wake-up wiggle before settling into idle. Edit BOOT_SEQUENCE above. */
    play_boot_sequence();

    s_idle_enabled = !DEV_DISABLE_IDLE;
    if (s_idle_task == NULL) {
        xTaskCreatePinnedToCore(idle_task, "servo_idle", 3072, NULL, 3, &s_idle_task, 0);
    }
    ESP_LOGI(TAG, "servo bus up, idle animation running");
    return ESP_OK;
}

void stackchan_servo_idle_set(bool enabled)
{
    s_idle_enabled = DEV_DISABLE_IDLE ? false : enabled;
}

void stackchan_servo_move(int yaw_deg, int pitch_deg, int speed)
{
    if (!s_bus_ready) return;
    s_idle_enabled = false;
    /* Map speed 0-1000 to a move time: fast=short. */
    speed = clamp_int(speed, 0, 1000);
    uint16_t move_ms = (uint16_t)(1400 - (speed * 1200) / 1000);  /* 1400ms .. 200ms */
    scs_write_pos(YAW_ID, yaw_deg_to_raw(yaw_deg), move_ms);
    vTaskDelay(pdMS_TO_TICKS(30));
    scs_write_pos(PITCH_ID, pitch_deg_to_raw(pitch_deg), move_ms);
    s_aim_yaw_deg = yaw_deg;
    s_aim_pitch_deg = pitch_deg;
}

void stackchan_servo_get_aim(int *yaw_deg, int *pitch_deg)
{
    if (yaw_deg)   *yaw_deg   = s_aim_yaw_deg;
    if (pitch_deg) *pitch_deg = s_aim_pitch_deg;
}

void stackchan_servo_home(void)
{
    if (!s_bus_ready) return;
    s_idle_enabled = false;
    scs_write_pos(YAW_ID, YAW_REST, 800);
    vTaskDelay(pdMS_TO_TICKS(30));
    scs_write_pos(PITCH_ID, PITCH_REST, 800);
    s_aim_yaw_deg = 0;
    s_aim_pitch_deg = 0;
}

/* How long the shutdown recenter is allowed to finish before torque is cut. */
#define SHUTDOWN_MOVE_MS 900

void stackchan_servo_shutdown(void)
{
    if (!s_bus_ready) return;
    /* Stop any wandering, glide back to dead center, let it arrive, then relax
     * torque so the head is safe to power off / handle. */
    s_idle_enabled = false;
    scs_write_pos(YAW_ID, YAW_REST, SHUTDOWN_MOVE_MS);
    vTaskDelay(pdMS_TO_TICKS(30));
    scs_write_pos(PITCH_ID, PITCH_REST, SHUTDOWN_MOVE_MS);
    vTaskDelay(pdMS_TO_TICKS(SHUTDOWN_MOVE_MS + 200));
    scs_enable_torque(YAW_ID, false);
    scs_enable_torque(PITCH_ID, false);
    ESP_LOGI(TAG, "servo shutdown: recentered + torque released");
}

void stackchan_servo_relax(void)
{
    if (!s_bus_ready) return;
    /* Light-idle relax: stop wandering and cut holding torque so the head draws
     * no current while sleeping. No recenter/glide (that would cost a ~1s move
     * and the head goes limp under gravity once torque is off anyway). This is
     * the same torque release as the shutdown path, reused per spec. */
    s_idle_enabled = false;
    scs_enable_torque(YAW_ID, false);
    scs_enable_torque(PITCH_ID, false);
    ESP_LOGI(TAG, "servo relax (idle): torque released");
}

void stackchan_servo_wake(void)
{
    if (!s_bus_ready) return;
    /* Re-enable torque and gently recenter so the presentation coordinator's
     * pose apply can drive the head again. Idle stays disabled (DEV switch). */
    scs_enable_torque(YAW_ID, true);
    scs_enable_torque(PITCH_ID, true);
    scs_write_pos(YAW_ID, YAW_REST, 600);
    vTaskDelay(pdMS_TO_TICKS(30));
    scs_write_pos(PITCH_ID, PITCH_REST, 600);
    ESP_LOGI(TAG, "servo wake: torque re-enabled + recenter");
}

/* ------------------------------- commands -------------------------------- */

static esp_err_t handle_servo_move(
    esp_openclaw_node_handle_t node, void *ctx,
    const char *params_json, size_t params_len,
    char **out_json, esp_openclaw_node_error_t *err)
{
    (void)node; (void)ctx; (void)params_len;
    cJSON *p = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &p, err), TAG, "params");

    cJSON *jy = cJSON_GetObjectItemCaseSensitive(p, "yaw");
    cJSON *jp = cJSON_GetObjectItemCaseSensitive(p, "pitch");
    cJSON *js = cJSON_GetObjectItemCaseSensitive(p, "speed");
    int yaw = cJSON_IsNumber(jy) ? jy->valueint : 0;
    int pitch = cJSON_IsNumber(jp) ? jp->valueint : 0;
    int speed = cJSON_IsNumber(js) ? js->valueint : 500;
    stackchan_servo_move(yaw, pitch, speed);

    cJSON *out = cJSON_CreateObject();
    if (!out) { cJSON_Delete(p); return ESP_ERR_NO_MEM; }
    cJSON_AddNumberToObject(out, "yaw", yaw);
    cJSON_AddNumberToObject(out, "pitch", pitch);
    cJSON_Delete(p);
    return esp_openclaw_node_example_take_json_payload(out, out_json);
}

static esp_err_t handle_servo_home(
    esp_openclaw_node_handle_t node, void *ctx,
    const char *params_json, size_t params_len,
    char **out_json, esp_openclaw_node_error_t *err)
{
    (void)node; (void)ctx; (void)params_json; (void)params_len; (void)err;
    stackchan_servo_home();
    cJSON *out = cJSON_CreateObject();
    if (!out) return ESP_ERR_NO_MEM;
    cJSON_AddBoolToObject(out, "home", true);
    return esp_openclaw_node_example_take_json_payload(out, out_json);
}

static esp_err_t handle_servo_idle(
    esp_openclaw_node_handle_t node, void *ctx,
    const char *params_json, size_t params_len,
    char **out_json, esp_openclaw_node_error_t *err)
{
    (void)node; (void)ctx; (void)params_len;
    cJSON *p = NULL;
    ESP_RETURN_ON_ERROR(
        esp_openclaw_node_example_parse_json_params(params_json, &p, err), TAG, "params");
    cJSON *on = cJSON_GetObjectItemCaseSensitive(p, "on");
    bool enable = (on == NULL) ? true : cJSON_IsTrue(on);
    stackchan_servo_idle_set(enable);

    cJSON *out = cJSON_CreateObject();
    if (!out) { cJSON_Delete(p); return ESP_ERR_NO_MEM; }
    cJSON_AddBoolToObject(out, "idle", enable);
    cJSON_Delete(p);
    return esp_openclaw_node_example_take_json_payload(out, out_json);
}

static esp_err_t handle_servo_shutdown(
    esp_openclaw_node_handle_t node, void *ctx,
    const char *params_json, size_t params_len,
    char **out_json, esp_openclaw_node_error_t *err)
{
    (void)node; (void)ctx; (void)params_json; (void)params_len; (void)err;
    stackchan_servo_shutdown();
    cJSON *out = cJSON_CreateObject();
    if (!out) return ESP_ERR_NO_MEM;
    cJSON_AddBoolToObject(out, "shutdown", true);
    return esp_openclaw_node_example_take_json_payload(out, out_json);
}

esp_err_t esp_openclaw_node_stackchan_register_servo_commands(esp_openclaw_node_handle_t node)
{
    static const esp_openclaw_node_command_t CMDS[] = {
        { .name = "servo.move", .handler = handle_servo_move, .context = NULL },
        { .name = "servo.home", .handler = handle_servo_home, .context = NULL },
        { .name = "servo.idle", .handler = handle_servo_idle, .context = NULL },
        { .name = "servo.shutdown", .handler = handle_servo_shutdown, .context = NULL },
    };
    ESP_RETURN_ON_ERROR(esp_openclaw_node_register_capability(node, "servo"), TAG, "cap servo");
    for (size_t i = 0; i < sizeof(CMDS) / sizeof(CMDS[0]); i++) {
        ESP_RETURN_ON_ERROR(esp_openclaw_node_register_command(node, &CMDS[i]), TAG, "reg cmd");
    }
    return ESP_OK;
}
