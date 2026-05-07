#ifndef __BSP_AS5600_H__
#define __BSP_AS5600_H__

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* AS5600 I2C 地址 (7-bit 0x36, 左移 1 位 = 0x6C) */
#define AS5600_ADDR_WRITE   0x6CU

/* AS5600 寄存器 */
#define AS5600_REG_RAW_ANGLE  0x0CU  /* 12-bit RAW 角度 (低4bit为AGC, 需屏蔽) */
#define AS5600_AVG_SAMPLES    5      /* 多次采样取平均以滤除磁编噪声 */

/* 步进电机参数: 16 细分, 步距角 1.8° */
#define GEAR_RATIO          1.0f
#define STEPS_PER_DEGREE    ((3200.0f * GEAR_RATIO) / 360.0f)
#define DEGREES_PER_STEP    (360.0f / (3200.0f * GEAR_RATIO))

/* 编码器实例结构体 */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    float zero_offset;   /* 零点偏移角度 (物理角度) */
    float angle_deg;     /* 当前角度 (0~360, 已减去零点偏移) */
} AS5600_t;

/* 三个全局编码器实例 */
extern AS5600_t Encoder_M1;
extern AS5600_t Encoder_M2;
extern AS5600_t Encoder_M3;

/* API */
void BSP_AS5600_Init(void);
bool BSP_AS5600_Update(AS5600_t *enc);
bool BSP_AS5600_SetZero(AS5600_t *enc);

#endif /* __BSP_AS5600_H__ */
