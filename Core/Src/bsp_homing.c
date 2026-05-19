#include "bsp_homing.h"
#include "bsp_stepper.h"
#include "main.h"
#include "common.h"
#include <stdio.h>

#define HOMING_STEP_DELAY_US   200
#define HOMING_MAX_STEPS       8000

/* 微秒级忙等待 (168MHz Cortex-M4, 粗略但够用) */
static void delay_us(uint32_t us)
{
    uint32_t cnt = us * 21;
    for (volatile uint32_t i = 0; i < cnt; i++) {
        __NOP();
    }
}

bool BSP_Homing_Execute(void)
{
    printf("[Homing] Starting...\r\n");

    /* Phase 1: M1/M2 CW, M3 CCW until each hits its limit switch */
    BSP_Stepper_SetDir(&Motor_M1, STEPPER_DIR_CW);
    BSP_Stepper_SetDir(&Motor_M2, STEPPER_DIR_CW);
    BSP_Stepper_SetDir(&Motor_M3, STEPPER_DIR_CCW);

    bool m1_done = false, m2_done = false, m3_done = false;
    uint32_t step = 0;

    while (!m1_done || !m2_done || !m3_done) {
        if (++step > HOMING_MAX_STEPS) {
            printf("[Homing] TIMEOUT: M1=%d M2=%d M3=%d\r\n", m1_done, m2_done, m3_done);
            return false;
        }

        if (!m1_done) {
            BSP_Stepper_Step(&Motor_M1);
            if (HAL_GPIO_ReadPin(M1_STOP_GPIO_Port, M1_STOP_Pin) == GPIO_PIN_RESET) {
                m1_done = true;
                printf("[Homing] M1 triggered at step %lu\r\n", step);
            }
        }

        if (!m2_done) {
            BSP_Stepper_Step(&Motor_M2);
            if (HAL_GPIO_ReadPin(M2_STOP_GPIO_Port, M2_STOP_Pin) == GPIO_PIN_RESET) {
                m2_done = true;
                printf("[Homing] M2 triggered at step %lu\r\n", step);
            }
        }

        if (!m3_done) {
            BSP_Stepper_Step(&Motor_M3);
            if (HAL_GPIO_ReadPin(M3_STOP_GPIO_Port, M3_STOP_Pin) == GPIO_PIN_RESET) {
                m3_done = true;
                printf("[Homing] M3 triggered at step %lu\r\n", step);
            }
        }

        delay_us(HOMING_STEP_DELAY_US);
    }

    printf("[Homing] All switches hit. Backing off M1=130 M2=30 M3=6 deg...\r\n");

    /* Phase 2: Reverse each motor by its specified angle */
    int32_t bo_m1 = DegToSteps(130.0f);
    int32_t bo_m2 = DegToSteps(30.0f);
    int32_t bo_m3 = DegToSteps(6.0f);

    BSP_Stepper_SetDir(&Motor_M1, STEPPER_DIR_CCW);
    BSP_Stepper_SetDir(&Motor_M2, STEPPER_DIR_CCW);
    BSP_Stepper_SetDir(&Motor_M3, STEPPER_DIR_CW);

    int32_t max_steps = bo_m1;
    if (bo_m2 > max_steps) max_steps = bo_m2;
    if (bo_m3 > max_steps) max_steps = bo_m3;

    for (int32_t i = 0; i < max_steps; i++) {
        if (i < bo_m1) BSP_Stepper_Step(&Motor_M1);
        if (i < bo_m2) BSP_Stepper_Step(&Motor_M2);
        if (i < bo_m3) BSP_Stepper_Step(&Motor_M3);
        delay_us(HOMING_STEP_DELAY_US);
    }

    printf("[Homing] Complete.\r\n");
    return true;
}
