/**
 * @file    bsp_ps2.h
 * @brief   PS2 controller interface via bit-bang SPI / PS2 閹靛鐒烘担宥嗗ⅵ SPI 閹恒儱褰?
 * @ingroup bsp
 *
 * Reads button state and analog joystick values in PS2 analog (red) mode / 鐠囪褰囬幐澶愭尦閻樿埖鈧礁鎷板Ο鈩冨珯閹藉洦娼岄崐?(PS2 缁俱垻浼呭Ο鈩冨珯濡€崇础)
 * Automatically attempts re-initialization on mode loss / 濡€崇础娑撱垹銇戦弮鎯板殰閸斻劌鐨剧拠鏇㈠櫢閺傛澘鍨垫慨瀣
 */

#ifndef __Drv_PS2_H__
#define __Drv_PS2_H__

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 閳光偓閳光偓 Button bitmasks (pressed = 0, released = 1) / 閹稿鎸虫担宥嗗负閻?(閹稿绗?0, 閺夋儳绱?1) 閳光偓閳光偓 */
#define PS2_BTN_SELECT   0x0100U
#define PS2_BTN_L3       0x0200U
#define PS2_BTN_R3       0x0400U
#define PS2_BTN_START    0x0800U
#define PS2_BTN_UP       0x1000U     /* D-pad up / 閺傜懓鎮滈柨顔荤瑐 */
#define PS2_BTN_RIGHT    0x2000U     /* D-pad right / 閺傜懓鎮滈柨顔煎礁 */
#define PS2_BTN_DOWN     0x4000U     /* D-pad down / 閺傜懓鎮滈柨顔荤瑓 */
#define PS2_BTN_LEFT     0x8000U     /* D-pad left / 閺傜懓鎮滈柨顔间箯 */
#define PS2_BTN_L2       0x0001U
#define PS2_BTN_R2       0x0002U
#define PS2_BTN_L1       0x0004U
#define PS2_BTN_R1       0x0008U
#define PS2_BTN_TRIANGLE 0x0010U
#define PS2_BTN_CIRCLE   0x0020U
#define PS2_BTN_CROSS    0x0040U
#define PS2_BTN_SQUARE   0x0080U

typedef struct {
    uint16_t buttons;           /* Button bitmask / 閹稿鎸虫担宥嗗负閻?*/
    uint8_t  LX;                /* Left stick X (0-255) / 瀹革附鎲為弶?X */
    uint8_t  LY;                /* Left stick Y (0-255) / 瀹革附鎲為弶?Y */
    uint8_t  RX;                /* Right stick X (0-255) / 閸欒櫕鎲為弶?X */
    uint8_t  RY;                /* Right stick Y (0-255) / 閸欒櫕鎲為弶?Y */
} Ps2Input_t;

void Drv_PS2_Init(void);
bool Drv_PS2_ReadData(Ps2Input_t *data);
bool Drv_PS2_IsAnalogMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __Drv_PS2_H__ */
