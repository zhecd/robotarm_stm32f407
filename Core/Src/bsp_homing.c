#include "bsp_homing.h"
#include "bsp_stepper.h"
#include "common.h"
#include <stdio.h>

/* 齿轮比 20:90 (电机:关节), 关节度 → 电机步数 */
#define JOINT_DEG_TO_STEPS(deg) ((int32_t)((deg) * (90.0f / 20.0f) * STEPS_PER_DEGREE))

static void delay_us(uint32_t us)
{
    uint32_t cycles = us * (SystemCoreClock / 1000000UL);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) {}
}

static void step_motor(Stepper_Dev_t *motor, uint32_t steps, uint32_t delay_us_val)
{
    for (uint32_t i = 0; i < steps; i++)
    {
        BSP_Stepper_Step(motor);
        delay_us(delay_us_val);
    }
}

/*
 * 上电回零序列:
 *   Phase1 - M1(CW)→PC10, M2(CW)→PC11, M3(CCW)→PC12 找限位
 *   Phase2 - M1反转100°(关节), M2反转10°(关节), M3反转10°(关节) 到达零点
 *   齿轮比 20:90 (电机:关节), 限位触发 = 低电平
 */
void BSP_Homing_Execute(void)
{
    printf("[Homing] Phase1: Finding limit switches...\r\n");
    printf("[Homing] M1(CW)->PC10  M2(CW)->PC11  M3(CCW)->PC12\r\n");

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    const uint32_t step_delay_us = 800;

    bool m1_done = false;
    bool m2_done = false;
    bool m3_done = false;

    BSP_Stepper_SetDir(&Motor_M1, STEPPER_DIR_CW);
    BSP_Stepper_SetDir(&Motor_M2, STEPPER_DIR_CW);
    BSP_Stepper_SetDir(&Motor_M3, STEPPER_DIR_CCW);

    uint32_t step_count = 0;

    while (!m1_done || !m2_done || !m3_done)
    {
        if (!m1_done)
        {
            if (HAL_GPIO_ReadPin(M1_STOP_GPIO_Port, M1_STOP_Pin) == GPIO_PIN_RESET)
            {
                m1_done = true;
                printf("[Homing] M1 limit hit at step %lu\r\n", step_count);
            }
            else
            {
                BSP_Stepper_Step(&Motor_M1);
            }
        }

        if (!m2_done)
        {
            if (HAL_GPIO_ReadPin(M2_STOP_GPIO_Port, M2_STOP_Pin) == GPIO_PIN_RESET)
            {
                m2_done = true;
                printf("[Homing] M2 limit hit at step %lu\r\n", step_count);
            }
            else
            {
                BSP_Stepper_Step(&Motor_M2);
            }
        }

        if (!m3_done)
        {
            if (HAL_GPIO_ReadPin(M3_STOP_GPIO_Port, M3_STOP_Pin) == GPIO_PIN_RESET)
            {
                m3_done = true;
                printf("[Homing] M3 limit hit at step %lu\r\n", step_count);
            }
            else
            {
                BSP_Stepper_Step(&Motor_M3);
            }
        }

        step_count++;
        delay_us(step_delay_us);

        if (step_count > 500000)
        {
            printf("[Homing] ERROR: Phase1 timeout!\r\n");
            goto homing_done;
        }
    }

    printf("[Homing] Phase2: Back-off to zero position...\r\n");

    /* M1 反转: 100°关节 → 100 × (90/20) × (3200/360) = 4000 步 */
    BSP_Stepper_SetDir(&Motor_M1, STEPPER_DIR_CCW);
    step_motor(&Motor_M1, JOINT_DEG_TO_STEPS(100.0f), step_delay_us);
    printf("[Homing] M1 backed off 100 deg (joint)\r\n");

    /* M2 反转: 25°关节 → 1000 步 */
    BSP_Stepper_SetDir(&Motor_M2, STEPPER_DIR_CCW);
    step_motor(&Motor_M2, JOINT_DEG_TO_STEPS(25.0f), step_delay_us);
    printf("[Homing] M2 backed off 25 deg (joint)\r\n");

    /* M3 反转: 10°关节 → 400 步 */
    BSP_Stepper_SetDir(&Motor_M3, STEPPER_DIR_CW);
    step_motor(&Motor_M3, JOINT_DEG_TO_STEPS(10.0f), step_delay_us);
    printf("[Homing] M3 backed off 10 deg (joint)\r\n");

homing_done:
    printf("[Homing] Complete. M1:%s M2:%s M3:%s\r\n",
           m1_done ? "OK" : "FAIL",
           m2_done ? "OK" : "FAIL",
           m3_done ? "OK" : "FAIL");
}
