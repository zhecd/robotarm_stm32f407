/**
 * @file    bsp_stepper.h
 * @brief   Stepper motor GPIO driver (step pulse + direction + enable) / 姝ヨ繘鐢垫満GPIO椹卞�?(鑴夊�?鏂瑰�?浣胯�?
 * @ingroup bsp
 *
 * Manages 3 stepper motor instances / 绠＄�?3 涓杩涚數鏈哄疄渚?
 *   - M1 (Base rotation / 搴曞骇鏃嬭浆): PA6 STEP, PD3 DIR
 *   - M2 (Shoulder / 鑲╁叧鑺?:       PA7 STEP, PD4 DIR
 *   - M3 (Elbow / 鑲樺叧鑺?:          PB0 STEP, PD5 DIR
 *   - EN (Global enable / 鍏ㄥ眬浣胯兘): PD2, active low / 浣庣數骞虫湁�?
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
    STEPPER_DIRECTION_CW  = 0,    /* Clockwise / 椤烘椂閽?*/
    STEPPER_DIRECTION_CCW = 1     /* Counter-clockwise / 閫嗘椂閽?*/
} StepperDirection_t;

typedef struct {
    GPIO_TypeDef       *step_port;          /* Step pin port / 姝ヨ繘寮曡剼绔�?*/
    uint16_t            step_pin;           /* Step pin number / 姝ヨ繘寮曡剼�?*/
    GPIO_TypeDef       *dir_port;           /* Direction pin port / 鏂瑰悜寮曡剼绔�?*/
    uint16_t            dir_pin;            /* Direction pin number / 鏂瑰悜寮曡剼�?*/
    StepperDirection_t        current_dir;        /* Current direction / 褰撳墠鏂瑰悜 */
    volatile int32_t    absolute_position;  /* Absolute position (steps) / 缁濆浣嶇疆(�? */
} StepperDevice_t;

void Drv_Stepper_Init(void);
bool Drv_Stepper_Enable(StepperDevice_t *motor, bool enable);
bool Drv_Stepper_SetDir(StepperDevice_t *motor, StepperDirection_t dir);
bool Drv_Stepper_Step(StepperDevice_t *motor);

StepperDevice_t *Drv_Stepper_GetM1(void);
StepperDevice_t *Drv_Stepper_GetM2(void);
StepperDevice_t *Drv_Stepper_GetM3(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_STEPPER_H__ */
