/*
 * SCCB (camera sensor control) transport for the M5Stack CoreS3 StackChan.
 *
 * === WHY THIS FILE IS NOT THE UPSTREAM sccb.c ===
 * The CoreS3 GC0308 camera's SCCB control lines are GPIO12 (SDA) / GPIO11 (SCL)
 * -- the SAME internal I2C bus (I2C_NUM_1) that the AXP2101 PMU, AW9523/PY32
 * expander, audio codecs and RGB LEDs share. That bus has exactly ONE owner:
 * M5Unified's M5.In_I2C, which LovyanGFX drives at the register/LL level (it
 * does NOT install the IDF legacy or new i2c driver). Upstream sccb.c would
 * either call i2c_driver_install() (legacy) or i2c_new_master_bus() (ng) on
 * that port, creating a SECOND concurrent bus owner -- the exact two-owner
 * race that caused false AXP2101 "power key -> shutdown" events and codec
 * begin() failures (see repo commit 39778be + memory notes).
 *
 * So instead of touching the I2C peripheral directly, every SCCB byte here is
 * routed through M5.In_I2C via the C shim in main/stackchan_camera.cpp
 * (stk_cam_sccb_*). M5 keeps sole ownership of the bus at ALL times; camera
 * register config is just more transactions serialized behind M5's own I2C
 * lock, exactly like the LED/codec/power paths. No bus release, no second
 * driver, so the camera cannot reintroduce the phantom-shutdown regression.
 *
 * The DVP pixel path (VSYNC/HREF/PCLK/D0..D7) is a separate parallel bus
 * (LCD_CAM peripheral) and is left entirely to the upstream esp32-camera
 * driver; only the sensor's I2C/SCCB control regs come through here.
 */

#include <stdint.h>
#include "sccb.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "sccb-m5";

/* Implemented in main/stackchan_camera.cpp on top of M5.In_I2C (the single
 * internal-I2C owner). Kept as plain C externs so this C translation unit does
 * not need the M5Unified C++ headers. */
extern int stk_cam_sccb_read8(uint8_t slv_addr, uint8_t reg);            /* byte, or -1 on NAK */
extern int stk_cam_sccb_write8(uint8_t slv_addr, uint8_t reg, uint8_t data); /* 0 ok / -1 */
extern int stk_cam_sccb_probe(uint8_t slv_addr);                          /* 0 ok / -1 */

/* M5 already owns and configured the bus; there is nothing to install/tear
 * down here. Accept whatever port/pins esp_camera passes and succeed. */
int SCCB_Init(int pin_sda, int pin_scl)
{
    (void)pin_sda;
    (void)pin_scl;
    ESP_LOGI(TAG, "SCCB routed through M5.In_I2C (shared-bus owner); no 2nd i2c driver");
    return ESP_OK;
}

int SCCB_Use_Port(int sccb_i2c_port)
{
    (void)sccb_i2c_port;
    return ESP_OK;
}

int SCCB_Deinit(void)
{
    return ESP_OK;
}

int SCCB_Probe(uint8_t slv_addr)
{
    return stk_cam_sccb_probe(slv_addr) == 0 ? ESP_OK : ESP_FAIL;
}

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    int v = stk_cam_sccb_read8(slv_addr, reg);
    if (v < 0) {
        ESP_LOGE(TAG, "SCCB_Read fail addr:0x%02x reg:0x%02x", slv_addr, reg);
        return 0;
    }
    return (uint8_t)v;
}

int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    if (stk_cam_sccb_write8(slv_addr, reg, data) != 0) {
        ESP_LOGE(TAG, "SCCB_Write fail addr:0x%02x reg:0x%02x data:0x%02x", slv_addr, reg, data);
        return -1;
    }
    return 0;
}

/* GC0308 uses only 8-bit register addressing, so the 16-bit variants are never
 * exercised on this board. Provide safe implementations anyway (hi byte of the
 * register is written as a separate address byte) so the driver still links and
 * a future 16-bit sensor would not silently misbehave. */
uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg)
{
    /* Not used by GC0308. Best-effort: treat low byte as the register. */
    int v = stk_cam_sccb_read8(slv_addr, (uint8_t)(reg & 0xFF));
    return v < 0 ? 0 : (uint8_t)v;
}

int SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data)
{
    return SCCB_Write(slv_addr, (uint8_t)(reg & 0xFF), data);
}

uint16_t SCCB_Read_Addr16_Val16(uint8_t slv_addr, uint16_t reg)
{
    int hi = stk_cam_sccb_read8(slv_addr, (uint8_t)(reg & 0xFF));
    return hi < 0 ? 0 : (uint16_t)hi;
}

int SCCB_Write_Addr16_Val16(uint8_t slv_addr, uint16_t reg, uint16_t data)
{
    return SCCB_Write(slv_addr, (uint8_t)(reg & 0xFF), (uint8_t)(data & 0xFF));
}
