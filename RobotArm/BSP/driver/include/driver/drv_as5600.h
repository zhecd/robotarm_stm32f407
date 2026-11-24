/**
 * @file drv_as5600.h
 * @brief AS5600 驱动：I2C 读取、圆周均值、软件多圈跟踪与置零。
 * @details 软件多圈依赖连续采样且相邻样本转角小于 180 deg；断电后必须重新回零建立参考。
 */

#ifndef __Drv_AS5600_H__
#define __Drv_AS5600_H__

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "robot_math.h"
#include "error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AS5600_ADDR_WRITE       0x6CU           /* I2C write address */
#define AS5600_REG_RAW_ANGLE    0x0CU           /* Raw angle register */
#define AS5600_AVG_SAMPLES      5               /* Averaging sample count */
#define AS5600_I2C_TIMEOUT_MS   10U             /* Bound a failed sensor read. */
#define AS5600_FAILURE_BACKOFF_MS 100U         /* 失败后暂停真实总线访问，避免每圈主循环重复阻塞。 */
#define AS5600_RECOVERY_INTERVAL_MS 1000U      /* 同一路恢复尝试最短间隔，禁止无限恢复。 */

typedef struct {
    I2C_HandleTypeDef *hi2c;                /* I2C handle */
    float              zero_offset;         /* 置零时的电机轴原始角度，deg。 */
    float              angle_deg;           /* Last continuous angle (deg) */
    float              raw_unwrapped;       /* Accumulated raw angle across turns */
    int32_t            turn_count;          /* Accumulated turn count */
    bool               initialized;         /* First valid sample has been received. */
    uint32_t last_hal_error;                 /* 最近失败的HAL错误位，成功不抹去故障证据。 */
    uint32_t recovery_attempts;              /* 已发起的有界总线恢复次数。 */
    uint32_t recovery_successes;             /* 恢复后真实重读成功次数，非仅初始化成功。 */
    uint32_t last_recovery_ms, last_failure_ms;
    bool recovery_attempted, retry_wait;
    uint32_t last_hal_status;                /* 最近失败的HAL返回值；HAL_BUSY需结合错误/句柄状态解释。 */
} As5600Device_t;

ErrorCode_t  Drv_AS5600_Init(void);
/** 完成一次圆周均值采样和软件解圈；通信失败不得当作零角度使用。 */
ErrorCode_t  Drv_AS5600_Update(As5600Device_t *enc);
/** 以当前电机姿态建立编码器零偏，不修改机构几何零点定义。 */
ErrorCode_t  Drv_AS5600_SetZero(As5600Device_t *enc);

/* Correct turn count using expected angle from theory steps.
    Call after long motion gaps where multiple turns may have been missed.
 */
void         Drv_AS5600_SyncTurn(As5600Device_t *enc, float expected_deg);

void         Drv_AS5600_PrintStatus(void);
int32_t      Drv_AS5600_GetSteps(As5600Device_t *enc);

As5600Device_t *Drv_AS5600_GetM1(void);
As5600Device_t *Drv_AS5600_GetM2(void);
As5600Device_t *Drv_AS5600_GetM3(void);

#ifdef __cplusplus
}
#endif

#endif /* __Drv_AS5600_H__ */
