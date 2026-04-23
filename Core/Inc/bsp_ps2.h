#ifndef __BSP_PS2_H
#define __BSP_PS2_H

#include "main.h"
#include <stdbool.h>

// PS2 按键映射掩码表
#define PS2_BTN_SELECT      0x0100
#define PS2_BTN_L3          0x0200
#define PS2_BTN_R3          0x0400
#define PS2_BTN_START       0x0800
#define PS2_BTN_UP          0x1000
#define PS2_BTN_RIGHT       0x2000
#define PS2_BTN_DOWN        0x4000
#define PS2_BTN_LEFT        0x8000

#define PS2_BTN_L2          0x0001
#define PS2_BTN_R2          0x0002
#define PS2_BTN_L1          0x0004
#define PS2_BTN_R1          0x0008
#define PS2_BTN_TRIANGLE    0x0010 // 绿三角
#define PS2_BTN_CIRCLE      0x0020 // 红圆
#define PS2_BTN_CROSS       0x0040 // 蓝叉
#define PS2_BTN_SQUARE      0x0080 // 粉方块

typedef struct {
    uint16_t buttons; // 按键状态（0为按下，1为松开）
    uint8_t  LX;      // 左摇杆 X 轴 (0~255，128为中心)
    uint8_t  LY;      // 左摇杆 Y 轴 (0~255，128为中心)
    uint8_t  RX;      // 右摇杆 X 轴 (0~255，128为中心)
    uint8_t  RY;      // 右摇杆 Y 轴 (0~255，128为中心)
} PS2_Data_t;

void BSP_PS2_Init(void);
bool BSP_PS2_ReadData(PS2_Data_t *ps2_data);

#endif