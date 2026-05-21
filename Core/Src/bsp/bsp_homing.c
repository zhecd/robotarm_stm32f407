/**
 * @file    bsp_homing.c
 * @brief   Limit-switch-based homing sequence implementation / 限位开关回零序列实现
 * @ingroup bsp
 *
 * Phase 1: Drive each motor until its limit switch triggers / 驱动各电机直到触碰限位开关
 * Phase 2: Back off by a predefined angle / 反向回退预设角度
 *
 * @note    Uses busy-wait delay; blocks for the duration of homing / 使用忙等待延时, 回零期间阻塞
 */

#include "bsp/bsp_homing.h"
#include "bsp/bsp_stepper.h"
#include "main.h"
#include "common.h"
#include <stdio.h>

/* ── Microsecond busy-wait delay / 微秒忙等待延时 ── */

static void delay_us(uint32_t us)
{
    uint32_t cnt = us * 21;
    for (volatile uint32_t i = 0; i < cnt; i++) { __NOP(); }
}

bool BSP_Homing_Execute(void)
{
    printf("[Homing] Starting... / 回零开始...\r\n");

    Stepper_Dev_t *m1 = BSP_Stepper_GetM1();
    Stepper_Dev_t *m2 = BSP_Stepper_GetM2();
    Stepper_Dev_t *m3 = BSP_Stepper_GetM3();

    /* Phase 1: M1/M2 CW, M3 CCW until each hits its limit switch / 阶段1: 各电机旋转至触碰限位开关 */
    BSP_Stepper_SetDir(m1, STEPPER_DIR_CW);
    BSP_Stepper_SetDir(m2, STEPPER_DIR_CW);
    BSP_Stepper_SetDir(m3, STEPPER_DIR_CCW);

    bool     m1_done = false, m2_done = false, m3_done = false;
    uint32_t step    = 0;

    while (!m1_done || !m2_done || !m3_done) {
        if (++step > HOMING_MAX_STEPS) {
            printf("[Homing] TIMEOUT / 超时: M1=%d M2=%d M3=%d\r\n", m1_done, m2_done, m3_done);
            return false;
        }

        if (!m1_done) {
            BSP_Stepper_Step(m1);
            if (HAL_GPIO_ReadPin(M1_STOP_GPIO_Port, M1_STOP_Pin) == GPIO_PIN_RESET) {
                m1_done = true;
                printf("[Homing] M1 triggered / M1 触发 at step %lu\r\n", step);
            }
        }
        if (!m2_done) {
            BSP_Stepper_Step(m2);
            if (HAL_GPIO_ReadPin(M2_STOP_GPIO_Port, M2_STOP_Pin) == GPIO_PIN_RESET) {
                m2_done = true;
                printf("[Homing] M2 triggered / M2 触发 at step %lu\r\n", step);
            }
        }
        if (!m3_done) {
            BSP_Stepper_Step(m3);
            if (HAL_GPIO_ReadPin(M3_STOP_GPIO_Port, M3_STOP_Pin) == GPIO_PIN_RESET) {
                m3_done = true;
                printf("[Homing] M3 triggered / M3 触发 at step %lu\r\n", step);
            }
        }
        delay_us(HOMING_STEP_DELAY_US);
    }

    printf("[Homing] All switches hit. Backing off / 全部触发, 开始回退 M1=%.0f M2=%.0f M3=%.0f deg...\r\n",
           HOMING_BACKOFF_M1_DEG, HOMING_BACKOFF_M2_DEG, HOMING_BACKOFF_M3_DEG);

    /* Phase 2: Reverse each motor by its specified angle / 阶段2: 反向回退指定角度 */
    int32_t bo_m1 = (int32_t)(HOMING_BACKOFF_M1_DEG * UNITS_PER_DEGREE);
    int32_t bo_m2 = (int32_t)(HOMING_BACKOFF_M2_DEG * UNITS_PER_DEGREE);
    int32_t bo_m3 = (int32_t)(HOMING_BACKOFF_M3_DEG * UNITS_PER_DEGREE);

    BSP_Stepper_SetDir(m1, STEPPER_DIR_CCW);
    BSP_Stepper_SetDir(m2, STEPPER_DIR_CCW);
    BSP_Stepper_SetDir(m3, STEPPER_DIR_CW);

    int32_t max_steps = bo_m1;
    if (bo_m2 > max_steps) max_steps = bo_m2;
    if (bo_m3 > max_steps) max_steps = bo_m3;

    for (int32_t i = 0; i < max_steps; i++) {
        if (i < bo_m1) BSP_Stepper_Step(m1);
        if (i < bo_m2) BSP_Stepper_Step(m2);
        if (i < bo_m3) BSP_Stepper_Step(m3);
        delay_us(HOMING_STEP_DELAY_US);
    }

    printf("[Homing] Complete. / 回零完成\r\n");
    return true;
}
