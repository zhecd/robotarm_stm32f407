/**
 * @file dev_gripper.h
 * @brief 夹爪设备：将设备动作委托给板级绑定与 PWM 舵机驱动。
 */
#ifndef ROBOTARM_DEV_GRIPPER_H
#define ROBOTARM_DEV_GRIPPER_H

#include <stdbool.h>
#include "gripper_status.h"

/** 初始化夹爪并执行原有闭合动作，仅供正常固件启动使用。 */
void Dev_Gripper_Init(void);
/** 正常启动的可检查初始化；会立即发出原闭合目标，不是无动作绑定。 */
bool Dev_Gripper_TryInit(void);
/** 返回底层目标发送结果；不向Service暴露TIM句柄，不代表实际到位。 */
bool Dev_Gripper_TryOpen(void);
bool Dev_Gripper_TryClose(void);
/* 原兼容入口保留；需要判断结果的Service必须使用Try接口。 */
void Dev_Gripper_Open(void);
void Dev_Gripper_Close(void);
void Dev_Gripper_Service(void);
/** 仅复制驱动记录，不初始化、不发出目标；调用方保护短快照。 */
void Dev_Gripper_GetStatus(GripperStatus_t *out);

#endif /* ROBOTARM_DEV_GRIPPER_H */
