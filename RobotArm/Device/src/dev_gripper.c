/**
 * @file dev_gripper.c
 * @brief 夹爪设备：将设备动作委托给板级绑定与 PWM 舵机驱动。
 */
#include "device/dev_gripper.h"
#include "driver/drv_servo.h"
#include "bsp/bsp_robot_board.h"

void Dev_Gripper_Init(void)
{
    (void)Dev_Gripper_TryInit();
}

bool Dev_Gripper_TryInit(void) { return BSP_RobotBoard_TryInitGripper(); }

bool Dev_Gripper_TryOpen(void) { return Drv_Servo_TryOpen(Drv_Servo_GetHandle()); }
bool Dev_Gripper_TryClose(void) { return Drv_Servo_TryClose(Drv_Servo_GetHandle()); }
void Dev_Gripper_Open(void) { (void)Dev_Gripper_TryOpen(); }
void Dev_Gripper_Close(void) { (void)Dev_Gripper_TryClose(); }
void Dev_Gripper_Service(void) { Drv_Servo_IdleStop(Drv_Servo_GetHandle()); }
void Dev_Gripper_GetStatus(GripperStatus_t *out)
{ Drv_Servo_GetStatus(Drv_Servo_GetHandle(),out); }
