/**
 * @file    bsp_as5600.h
 * @brief   AS5600 magnetic encoder driver (I2C, 12-bit angle) / AS5600 磁编码器驱动 (I2C, 12位角度)
 * @ingroup bsp
 *
 * Three independent encoder instances on I2C1/I2C2/I2C3 / 三路独立编码器挂载 I2C1/I2C2/I2C3
 * Uses 5-sample averaging for noise reduction / 5次采样平均降噪
 */

#ifndef __BSP_AS5600_H__
#define __BSP_AS5600_H__

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "common.h"
#include "error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AS5600_ADDR_WRITE       0x6CU           /* I2C write address / I2C 写地址 */
#define AS5600_REG_RAW_ANGLE    0x0CU           /* Raw angle register / 原始角度寄存器 */
#define AS5600_AVG_SAMPLES      5               /* Averaging sample count / 平均采样次数 */

typedef struct {
    I2C_HandleTypeDef *hi2c;                /* I2C handle / I2C 句柄 */
    float              zero_offset;         /* Physical zero point (deg) / 物理零点 */
    float              angle_deg;           /* Current angle, normalized to (-180, 180] / 当前角度, 归一化 */
} AS5600_Dev_t;

ErrorCode_t  BSP_AS5600_Init(void);
ErrorCode_t  BSP_AS5600_Update(AS5600_Dev_t *enc);
ErrorCode_t  BSP_AS5600_SetZero(AS5600_Dev_t *enc);
void         BSP_AS5600_PrintStatus(void);
int32_t      BSP_AS5600_GetSteps(AS5600_Dev_t *enc);

AS5600_Dev_t *BSP_AS5600_GetM1(void);
AS5600_Dev_t *BSP_AS5600_GetM2(void);
AS5600_Dev_t *BSP_AS5600_GetM3(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_AS5600_H__ */
