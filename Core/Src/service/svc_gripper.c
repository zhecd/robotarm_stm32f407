/**
 * @file    ctrl_gripper.c
 * @brief   Gripper servo control implementation / 婢跺湱鍩呴懜鍨簚閹貉冨煑鐎圭偟骞? * @ingroup control
 */

#include "service/svc_gripper.h"
#include "device/dev_gripper.h"

void Svc_Gripper_Init(TIM_HandleTypeDef *htim)
{
    Dev_Gripper_Init(htim, TIM_CHANNEL_2);
}

void Svc_Gripper_Open(void)
{
    Dev_Gripper_Open();
}

void Svc_Gripper_Close(void)
{
    Dev_Gripper_Close();
}

void Svc_Gripper_IdleStop(void)
{
    Dev_Gripper_Service();
}
