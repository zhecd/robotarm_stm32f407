#ifndef __BSP_AS5600_H__
#define __BSP_AS5600_H__

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "common.h"

/* ── 硬件常量 ── */
#define AS5600_ADDR_WRITE       0x6CU           /* 7-bit 0x36 << 1 */
#define AS5600_REG_RAW_ANGLE    0x0CU           /* 12-bit RAW 角度 */
#define AS5600_AVG_SAMPLES      5               /* 多次采样取平均 */

/* ── 编码器实例 ── */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    float zero_offset;          /* 物理零点 (度) */
    float angle_deg;            /* 当前角度, 归一化到 (-180, 180] */
} AS5600_t;

extern AS5600_t Encoder_M1;
extern AS5600_t Encoder_M2;
extern AS5600_t Encoder_M3;

/* ── API ── */
void BSP_AS5600_Init(void);
bool BSP_AS5600_Update(AS5600_t *enc);
bool BSP_AS5600_SetZero(AS5600_t *enc);
void BSP_AS5600_PrintStatus(void);              /* 1Hz 编码器状态上报 */
int32_t BSP_AS5600_GetSteps(AS5600_t *enc);     /* 当前角度 → 微步数 */

#endif
