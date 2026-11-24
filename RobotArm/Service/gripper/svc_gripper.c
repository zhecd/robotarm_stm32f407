/**
 * @file svc_gripper.c
 * @brief 夹爪服务：表达打开、关闭和保持结束后的停 PWM 意图。
 */

#include "service/svc_gripper.h"
#include "device/dev_gripper.h"
#include "platform_critical.h"
#include "motion_service.h"

bool Svc_Gripper_Open(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    /* 许可与启动共享短临界区；拒绝不访问设备，启动失败逐层返回false。 */
    bool accepted = MotionService_IsActionAllowed() && Dev_Gripper_TryOpen();
    PlatformCritical_Exit(key);
    return accepted;
}

bool Svc_Gripper_Close(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    bool accepted = MotionService_IsActionAllowed() && Dev_Gripper_TryClose();
    PlatformCritical_Exit(key);
    return accepted;
}

void Svc_Gripper_IdleStop(void)
{
    Dev_Gripper_Service();
}

void Svc_Gripper_GetStatus(GripperStatus_t *out)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    Dev_Gripper_GetStatus(out);
    PlatformCritical_Exit(key);
}
