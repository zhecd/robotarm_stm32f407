/**
 * @file    bsp_gripper.h
 * @brief   PWM servo gripper driver (TIM2_CH2, PA1) / PWM 舵机夹爪驱动 (TIM2_CH2, PA1)
 * @ingroup bsp
 *
 * Controls a standard RC servo (500-2500 us pulse, 50 Hz) / 控制标准 RC 舵机 (500-2500us 脉宽, 50Hz)
 */

#ifndef __BSP_GRIPPER_H__
#define __BSP_GRIPPER_H__

#include "main.h"
#include "robot_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GRIPPER_STATE_UNKNOWN = 0,      /* Unknown state / 未知状态 */
    GRIPPER_STATE_OPEN,             /* Fully open / 完全张开 */
    GRIPPER_STATE_CLOSE,            /* Fully closed / 完全闭合 */
    GRIPPER_STATE_CUSTOM_ANGLE      /* Custom angle / 自定义角度 */
} GripperState_t;

typedef struct {
    TIM_HandleTypeDef *htim;        /* Timer handle / 定时器句柄 */
    uint32_t           channel;     /* Timer channel / 定时器通道 */
    float              cur_angle;   /* Current angle (deg) / 当前角度 */
    GripperState_t     state;       /* Current state / 当前状态 */
    uint32_t           hold_start;  /* Last angle-change tick for auto-off */
} Gripper_Dev_t;

void BSP_Gripper_Init(Gripper_Dev_t *grp, TIM_HandleTypeDef *htim, uint32_t channel);
void BSP_Gripper_SetAngle(Gripper_Dev_t *grp, float angle);
void BSP_Gripper_Open(Gripper_Dev_t *grp);
void BSP_Gripper_Close(Gripper_Dev_t *grp);
void BSP_Gripper_Stop(Gripper_Dev_t *grp);
void BSP_Gripper_IdleStop(Gripper_Dev_t *grp);

Gripper_Dev_t *BSP_Gripper_GetHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_GRIPPER_H__ */
