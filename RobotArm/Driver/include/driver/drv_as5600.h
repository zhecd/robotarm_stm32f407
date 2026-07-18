/**
 * @file    bsp_as5600.h
 * @brief   AS5600 magnetic encoder driver (I2C, 12-bit angle) / AS5600 纾佺紪鐮佸櫒椹卞�?(I2C, 12浣嶈搴?
 * @ingroup bsp
 *
 * Three independent encoder instances on I2C1/I2C2/I2C3 / 涓夎矾鐙珛缂栫爜鍣ㄦ寕�?I2C1/I2C2/I2C3
 * Uses 5-sample averaging for noise reduction / 5娆￠噰鏍峰钩鍧囬檷鍣?
 *
 * Multi-turn resolution: callers pass expected_deg (from theory steps) to
 * resolve which turn the single-turn sensor is on.
 * 澶氬湀瑙ｆ�? 璋冪敤鑰呬紶�?expected_deg (鏉ヨ嚜鐞嗚姝ユ�? 纭畾鍗曞湀浼犳劅鍣ㄦ墍鍦ㄥ湀鏁般�?
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

#define AS5600_ADDR_WRITE       0x6CU           /* I2C write address / I2C 鍐欏湴鍧�?*/
#define AS5600_REG_RAW_ANGLE    0x0CU           /* Raw angle register / 鍘熷瑙掑害瀵勫瓨鍣?*/
#define AS5600_AVG_SAMPLES      5               /* Averaging sample count / 骞冲潎閲囨牱娆℃�?*/
#define AS5600_I2C_TIMEOUT_MS   10U             /* Bound a failed sensor read. */

typedef struct {
    I2C_HandleTypeDef *hi2c;                /* I2C handle / I2C 鍙ユ�?*/
    float              zero_offset;         /* Physical zero point (deg) / 鐗╃悊闆剁偣 */
    float              angle_deg;           /* Last continuous angle (deg) / 鏈€杩戣繛缁�?*/
    float              raw_unwrapped;       /* Accumulated raw angle across turns / 璺ㄥ湀绱Н鍘熷瑙掑害 */
    int32_t            turn_count;          /* Accumulated turn count / 绱Н鍦堟�?*/
    bool               initialized;         /* First valid sample has been received. */
} As5600Device_t;

ErrorCode_t  Drv_AS5600_Init(void);
ErrorCode_t  Drv_AS5600_Update(As5600Device_t *enc);
ErrorCode_t  Drv_AS5600_SetZero(As5600Device_t *enc);

/** Correct turn count using expected angle from theory steps.
    Call after long motion gaps where multiple turns may have been missed.
    鐢ㄧ悊璁烘鏁颁慨姝ｅ鍦堣鏁? 鍦ㄩ暱鏃堕棿杩愬姩闂撮殭鍚庤皟鐢ㄣ€?*/
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
