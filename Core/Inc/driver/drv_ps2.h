/**
 * @file    bsp_ps2.h
 * @brief   PS2 controller interface via bit-bang SPI / PS2 手柄位打 SPI 接口
 * @ingroup bsp
 *
 * Reads button state and analog joystick values in PS2 analog (red) mode / 读取按钮状态和模拟摇杆值 (PS2 红灯模拟模式)
 * Automatically attempts re-initialization on mode loss / 模式丢失时自动尝试重新初始化
 */

#ifndef __BSP_PS2_H__
#define __BSP_PS2_H__

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Button bitmasks (pressed = 0, released = 1) / 按钮位掩码 (按下=0, 松开=1) ── */
#define PS2_BTN_SELECT   0x0100U
#define PS2_BTN_L3       0x0200U
#define PS2_BTN_R3       0x0400U
#define PS2_BTN_START    0x0800U
#define PS2_BTN_UP       0x1000U     /* D-pad up / 方向键上 */
#define PS2_BTN_RIGHT    0x2000U     /* D-pad right / 方向键右 */
#define PS2_BTN_DOWN     0x4000U     /* D-pad down / 方向键下 */
#define PS2_BTN_LEFT     0x8000U     /* D-pad left / 方向键左 */
#define PS2_BTN_L2       0x0001U
#define PS2_BTN_R2       0x0002U
#define PS2_BTN_L1       0x0004U
#define PS2_BTN_R1       0x0008U
#define PS2_BTN_TRIANGLE 0x0010U
#define PS2_BTN_CIRCLE   0x0020U
#define PS2_BTN_CROSS    0x0040U
#define PS2_BTN_SQUARE   0x0080U

typedef struct {
    uint16_t buttons;           /* Button bitmask / 按钮位掩码 */
    uint8_t  LX;                /* Left stick X (0-255) / 左摇杆 X */
    uint8_t  LY;                /* Left stick Y (0-255) / 左摇杆 Y */
    uint8_t  RX;                /* Right stick X (0-255) / 右摇杆 X */
    uint8_t  RY;                /* Right stick Y (0-255) / 右摇杆 Y */
} PS2_Data_t;

void BSP_PS2_Init(void);
bool BSP_PS2_ReadData(PS2_Data_t *data);
bool BSP_PS2_IsAnalogMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PS2_H__ */
