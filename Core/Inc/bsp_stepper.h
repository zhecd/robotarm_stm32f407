#ifndef BSP_STEPPER_H
#define BSP_STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    STEPPER_DIR_CW = 0,
    STEPPER_DIR_CCW = 1
} StepperDir_t;

typedef struct
{
    GPIO_TypeDef *step_port;
    uint16_t step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    StepperDir_t current_dir;
    volatile int32_t absolute_position;
} Stepper_Dev_t;

void BSP_Stepper_Init(void);
bool BSP_Stepper_Enable(Stepper_Dev_t *motor, bool enable);
bool BSP_Stepper_SetDir(Stepper_Dev_t *motor, StepperDir_t dir);
bool BSP_Stepper_Step(Stepper_Dev_t *motor);

extern Stepper_Dev_t Motor_M1;
extern Stepper_Dev_t Motor_M2;
extern Stepper_Dev_t Motor_M3;

#ifdef __cplusplus
}
#endif

#endif /* BSP_STEPPER_H */
