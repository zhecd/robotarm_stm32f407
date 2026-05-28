/**
 * @file    bsp_gripper.c
 * @brief   PWM servo gripper driver implementation / PWM 舵机夹爪驱动实现
 * @ingroup bsp
 */

#include "bsp/bsp_gripper.h"
#include "common.h"

static Gripper_Dev_t s_gripper;

void BSP_Gripper_Init(Gripper_Dev_t *grp, TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (!grp || !htim) return;
    grp->htim    = htim;
    grp->channel = channel;
    grp->state   = GRIPPER_STATE_UNKNOWN;
    BSP_Gripper_Open(grp);      /* Default to open / 默认张开 */
}

void BSP_Gripper_SetAngle(Gripper_Dev_t *grp, float angle)
{
    if (!grp) return;

    /* Software angle limit / 软件角度限幅 */
    float safe_angle  = CLAMP(angle, 0.0f, 180.0f);
    /* Linear mapping: angle -> pulse width / 线性映射: 角度->脉宽 */
    float pulse_width = GRIPPER_SERVO_MIN_US
                      + (safe_angle / 180.0f) * (GRIPPER_SERVO_MAX_US - GRIPPER_SERVO_MIN_US);

    HAL_TIM_PWM_Start(grp->htim, grp->channel);
    __HAL_TIM_SET_COMPARE(grp->htim, grp->channel, (uint32_t)pulse_width);

    grp->cur_angle = safe_angle;
    grp->state     = GRIPPER_STATE_CUSTOM_ANGLE;
}

void BSP_Gripper_Open(Gripper_Dev_t *grp)
{
    if (grp->state != GRIPPER_STATE_OPEN) {
        BSP_Gripper_SetAngle(grp, GRIPPER_ANGLE_CLOSE);
        grp->state = GRIPPER_STATE_OPEN;
    }
}

void BSP_Gripper_Close(Gripper_Dev_t *grp)
{
    if (grp->state != GRIPPER_STATE_CLOSE) {
        BSP_Gripper_SetAngle(grp, GRIPPER_ANGLE_OPEN);
        grp->state = GRIPPER_STATE_CLOSE;
    }
}

void BSP_Gripper_Stop(Gripper_Dev_t *grp)
{
    if (!grp) return;
    HAL_TIM_PWM_Stop(grp->htim, grp->channel);
    grp->state = GRIPPER_STATE_UNKNOWN;
}

Gripper_Dev_t *BSP_Gripper_GetHandle(void)
{
    return &s_gripper;
}
