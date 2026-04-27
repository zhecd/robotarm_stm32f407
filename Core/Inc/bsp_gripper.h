#ifndef BSP_GRIPPER_H
#define BSP_GRIPPER_H

#include "main.h"

#define GRIPPER_SERVO_MIN_US 500.0f
#define GRIPPER_SERVO_MAX_US 2500.0f
#define GRIPPER_ANGLE_OPEN   50.0f
#define GRIPPER_ANGLE_CLOSE  130.0f

typedef enum
{
    GRIPPER_STATE_UNKNOWN = 0,
    GRIPPER_STATE_OPEN,
    GRIPPER_STATE_CLOSE,
    GRIPPER_STATE_CUSTOM_ANGLE
} GripperState_t;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float cur_angle;
    GripperState_t state;
} Gripper_HandleTypeDef;

extern Gripper_HandleTypeDef hgripper;

void BSP_Gripper_Init(Gripper_HandleTypeDef *hgrp, TIM_HandleTypeDef *htim, uint32_t channel);
void BSP_Gripper_SetAngle(Gripper_HandleTypeDef *hgrp, float angle);
void BSP_Gripper_Open(Gripper_HandleTypeDef *hgrp);
void BSP_Gripper_Close(Gripper_HandleTypeDef *hgrp);
void BSP_Gripper_Stop(Gripper_HandleTypeDef *hgrp);

#endif /* BSP_GRIPPER_H */
