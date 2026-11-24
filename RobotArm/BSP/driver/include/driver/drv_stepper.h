/**
 * @file drv_stepper.h
 * @brief STEP/DIR 驱动：方向、单次脉冲与三轴共用使能。
 * @details NOP 数量不等同于已验证的 STEP 脉宽，实际脉宽和 DIR 建立时间须后续实机测量。
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
    STEPPER_DIRECTION_CW  = 0,    /* 顺时针方向标志，实际方向需按接线核对。 */
    STEPPER_DIRECTION_CCW = 1     /* Counter-clockwise */
} StepperDirection_t;

typedef struct {
    GPIO_TypeDef       *step_port;          /* Step pin port */
    uint16_t            step_pin;           /* Step pin number */
    GPIO_TypeDef       *dir_port;           /* Direction pin port */
    uint16_t            dir_pin;            /* Direction pin number */
    StepperDirection_t        current_dir;        /* 当前方向标志。 */
    volatile int32_t    absolute_position;  /* Absolute position (steps) */
} StepperDevice_t;

void Drv_Stepper_Init(void);
/* All axes share the same active-low EN pin on this board. */
bool Drv_Stepper_EnableAll(bool enable);
bool Drv_Stepper_SetDir(StepperDevice_t *motor, StepperDirection_t dir);
/** 输出一次 STEP 并累加驱动层计数；脉冲时序属于后续硬件验收项目。 */
bool Drv_Stepper_Step(StepperDevice_t *motor);
/** 仅写 STEP 电平，不累加位置；供禁用驱动的定时波形验收使用。 */
bool Drv_Stepper_WriteStepLevel(StepperDevice_t *motor, bool high);

StepperDevice_t *Drv_Stepper_GetM1(void);
StepperDevice_t *Drv_Stepper_GetM2(void);
StepperDevice_t *Drv_Stepper_GetM3(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_STEPPER_H__ */
