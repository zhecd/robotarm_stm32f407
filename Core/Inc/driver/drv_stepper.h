/**
 * @file    bsp_stepper.h
 * @brief   Stepper motor GPIO driver (step pulse + direction + enable) / 步进电机GPIO驱动 (脉冲+方向+使能)
 * @ingroup bsp
 *
 * Manages 3 stepper motor instances / 管理 3 个步进电机实例:
 *   - M1 (Base rotation / 底座旋转): PA6 STEP, PD3 DIR
 *   - M2 (Shoulder / 肩关节):       PA7 STEP, PD4 DIR
 *   - M3 (Elbow / 肘关节):          PB0 STEP, PD5 DIR
 *   - EN (Global enable / 全局使能): PD2, active low / 低电平有效
 */

#ifndef __BSP_STEPPER_H__
#define __BSP_STEPPER_H__

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STEPPER_DIR_CW  = 0,    /* Clockwise / 顺时针 */
    STEPPER_DIR_CCW = 1     /* Counter-clockwise / 逆时针 */
} StepperDir_t;

typedef struct {
    GPIO_TypeDef       *step_port;          /* Step pin port / 步进引脚端口 */
    uint16_t            step_pin;           /* Step pin number / 步进引脚号 */
    GPIO_TypeDef       *dir_port;           /* Direction pin port / 方向引脚端口 */
    uint16_t            dir_pin;            /* Direction pin number / 方向引脚号 */
    StepperDir_t        current_dir;        /* Current direction / 当前方向 */
    volatile int32_t    absolute_position;  /* Absolute position (steps) / 绝对位置(步) */
} Stepper_Dev_t;

void BSP_Stepper_Init(void);
bool BSP_Stepper_Enable(Stepper_Dev_t *motor, bool enable);
bool BSP_Stepper_SetDir(Stepper_Dev_t *motor, StepperDir_t dir);
bool BSP_Stepper_Step(Stepper_Dev_t *motor);

Stepper_Dev_t *BSP_Stepper_GetM1(void);
Stepper_Dev_t *BSP_Stepper_GetM2(void);
Stepper_Dev_t *BSP_Stepper_GetM3(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_STEPPER_H__ */
