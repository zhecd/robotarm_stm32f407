/**
 * @file drv_ps2.h
 * @brief PS2 驱动：GPIO 模拟串行时序、模拟模式初始化与数据包检查。
 */

#ifndef __Drv_PS2_H__
#define __Drv_PS2_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Button bitmasks (pressed = 0, released = 1) */
#define PS2_BTN_SELECT   0x0100U
#define PS2_BTN_L3       0x0200U
#define PS2_BTN_R3       0x0400U
#define PS2_BTN_START    0x0800U
#define PS2_BTN_UP       0x1000U     /* D-pad up */
#define PS2_BTN_RIGHT    0x2000U     /* D-pad right */
#define PS2_BTN_DOWN     0x4000U     /* D-pad down */
#define PS2_BTN_LEFT     0x8000U     /* D-pad left */
#define PS2_BTN_L2       0x0001U
#define PS2_BTN_R2       0x0002U
#define PS2_BTN_L1       0x0004U
#define PS2_BTN_R1       0x0008U
#define PS2_BTN_TRIANGLE 0x0010U
#define PS2_BTN_CIRCLE   0x0020U
#define PS2_BTN_CROSS    0x0040U
#define PS2_BTN_SQUARE   0x0080U

typedef struct {
    uint16_t buttons;           /* Button bitmask */
    uint8_t  LX;                /* 左摇杆 X 原始值，0..255。 */
    uint8_t  LY;                /* 左摇杆 Y 原始值，0..255。 */
    uint8_t  RX;                /* 右摇杆 X 原始值，0..255。 */
    uint8_t  RY;                /* 右摇杆 Y 原始值，0..255。 */
} Ps2Input_t;

/* 仅取软件已收数据，不访问总线、不重新轮询。用于串口/调试器事后诊断。
 * first_invalid_*只锁存见过有效帧后的第一个异常，避免后续样本覆盖现场。 */
typedef enum {
    PS2_READ_CONFIGURING=0,PS2_READ_VALID=1,PS2_READ_BAD_HEADER=2,
    PS2_READ_DIGITAL=3,PS2_READ_UNSUPPORTED_MODE=4
} Ps2ReadStatus_t;
typedef struct {
    uint32_t frames;
    uint32_t valid_frames;
    uint32_t invalid_frames;
    uint32_t last_invalid_ms;
    uint32_t first_invalid_ms;
    uint8_t last_status;
    uint8_t config_phase;
    uint8_t first_invalid_recorded;
    uint8_t first_invalid_status;
    uint8_t last_frame[9];
    uint8_t first_invalid_frame[9];
} Ps2Diagnostics_t;

/* Init不等待；主循环继续调用ReadData推进配置，尚未就绪返回false。 */
void Drv_PS2_Init(void);
bool Drv_PS2_ReadData(Ps2Input_t *data);
bool Drv_PS2_IsAnalogMode(void);
/* 主循环上下文复制；ISR不更新本记录。传NULL无副作用，Init开启新诊断会话。 */
void Drv_PS2_GetDiagnostics(Ps2Diagnostics_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __Drv_PS2_H__ */
