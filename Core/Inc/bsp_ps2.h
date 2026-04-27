#ifndef BSP_PS2_H
#define BSP_PS2_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define PS2_BTN_SELECT   0x0100U
#define PS2_BTN_L3       0x0200U
#define PS2_BTN_R3       0x0400U
#define PS2_BTN_START    0x0800U
#define PS2_BTN_UP       0x1000U
#define PS2_BTN_RIGHT    0x2000U
#define PS2_BTN_DOWN     0x4000U
#define PS2_BTN_LEFT     0x8000U

#define PS2_BTN_L2       0x0001U
#define PS2_BTN_R2       0x0002U
#define PS2_BTN_L1       0x0004U
#define PS2_BTN_R1       0x0008U
#define PS2_BTN_TRIANGLE 0x0010U
#define PS2_BTN_CIRCLE   0x0020U
#define PS2_BTN_CROSS    0x0040U
#define PS2_BTN_SQUARE   0x0080U

typedef struct
{
    uint16_t buttons;
    uint8_t LX;
    uint8_t LY;
    uint8_t RX;
    uint8_t RY;
} PS2_Data_t;

void BSP_PS2_Init(void);
bool BSP_PS2_ReadData(PS2_Data_t *ps2_data);
bool BSP_PS2_IsAnalogMode(void);

#endif /* BSP_PS2_H */
