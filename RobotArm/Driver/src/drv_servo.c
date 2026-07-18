/**
 * @file    bsp_gripper.c
 * @brief   PWM servo gripper driver implementation / PWM 閼稿灚婧€婢跺湱鍩呮す鍗炲З鐎圭偟骞? * @ingroup bsp
 */

#include "driver/drv_servo.h"
#include "robot_math.h"

static ServoDevice_t s_gripper;

void Drv_Servo_Init(ServoDevice_t *grp, TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (!grp || !htim) return;
    grp->htim       = htim;
    grp->channel    = channel;
    grp->state      = SERVO_STATE_UNKNOWN;
    grp->hold_start = 0;
    Drv_Servo_Close(grp);
}

void Drv_Servo_SetAngle(ServoDevice_t *grp, float angle)
{
    if (!grp) return;

    float safe_angle  = CLAMP(angle, 0.0f, 180.0f);
    float pulse_width = GRIPPER_SERVO_MIN_US
                      + (safe_angle / 180.0f) * (GRIPPER_SERVO_MAX_US - GRIPPER_SERVO_MIN_US);

    HAL_TIM_PWM_Start(grp->htim, grp->channel);
    __HAL_TIM_SET_COMPARE(grp->htim, grp->channel, (uint32_t)pulse_width);

    grp->cur_angle  = safe_angle;
    grp->state      = SERVO_STATE_CUSTOM_ANGLE;
    grp->hold_start = HAL_GetTick();
}

void Drv_Servo_Open(ServoDevice_t *grp)
{
    if (grp->state != SERVO_STATE_OPEN) {
        Drv_Servo_SetAngle(grp, GRIPPER_ANGLE_CLOSE);
        grp->state = SERVO_STATE_OPEN;
    }
}

void Drv_Servo_Close(ServoDevice_t *grp)
{
    if (grp->state != SERVO_STATE_CLOSE) {
        Drv_Servo_SetAngle(grp, GRIPPER_ANGLE_OPEN);
        grp->state = SERVO_STATE_CLOSE;
    }
}

void Drv_Servo_Stop(ServoDevice_t *grp)
{
    if (!grp) return;
    HAL_TIM_PWM_Stop(grp->htim, grp->channel);
    grp->hold_start = 0;
    grp->state      = SERVO_STATE_UNKNOWN;
}

void Drv_Servo_IdleStop(ServoDevice_t *grp)
{
    if (!grp || grp->hold_start == 0) return;
    if (HAL_GetTick() - grp->hold_start >= GRIPPER_HOLD_MS) {
        HAL_TIM_PWM_Stop(grp->htim, grp->channel);
        grp->hold_start = 0;
        grp->state      = SERVO_STATE_UNKNOWN;
    }
}

ServoDevice_t *Drv_Servo_GetHandle(void)
{
    return &s_gripper;
}
