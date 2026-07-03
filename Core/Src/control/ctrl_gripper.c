/**
 * @file    ctrl_gripper.c
 * @brief   Gripper servo control implementation / 夹爪舵机控制实现
 * @ingroup control
 */

#include "control/ctrl_gripper.h"
#include "bsp/bsp_gripper.h"

void Ctrl_Gripper_Init(void)
{
    extern TIM_HandleTypeDef htim2;
    BSP_Gripper_Init(BSP_Gripper_GetHandle(), &htim2, TIM_CHANNEL_2);
}

void Ctrl_Gripper_Open(void)
{
    BSP_Gripper_Open(BSP_Gripper_GetHandle());
}

void Ctrl_Gripper_Close(void)
{
    BSP_Gripper_Close(BSP_Gripper_GetHandle());
}

void Ctrl_Gripper_IdleStop(void)
{
    BSP_Gripper_IdleStop(BSP_Gripper_GetHandle());
}
