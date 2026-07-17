/** @file dev_gripper.c @brief Gripper device adapter. */
#include "device/dev_gripper.h"
#include "driver/drv_servo.h"

void Dev_Gripper_Init(TIM_HandleTypeDef *timer, uint32_t channel)
{
    Drv_Servo_Init(Drv_Servo_GetHandle(), timer, channel);
}

void Dev_Gripper_Open(void)  { Drv_Servo_Open(Drv_Servo_GetHandle()); }
void Dev_Gripper_Close(void) { Drv_Servo_Close(Drv_Servo_GetHandle()); }
void Dev_Gripper_Service(void) { Drv_Servo_IdleStop(Drv_Servo_GetHandle()); }
