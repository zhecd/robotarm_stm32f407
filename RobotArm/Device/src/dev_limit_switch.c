/**
 * @file dev_limit_switch.c
 * @brief 限位设备：以逻辑轴查询开关，GPIO 端口和有效电平由 BSP 解释。
 */
#include "device/dev_limit_switch.h"
#include "bsp/bsp_robot_board.h"

bool Dev_LimitSwitch_IsTriggered(DevJointId_t joint)
{
    return (unsigned)joint < DEV_JOINT_COUNT && BSP_RobotBoard_IsLimitTriggered((uint8_t)joint);
}
void Dev_LimitSwitch_MaskDiagnosticInterrupt(DevJointId_t joint)
{
    Dev_LimitSwitch_MaskInterrupt(joint);
}
void Dev_LimitSwitch_MaskInterrupt(DevJointId_t joint)
{
    if ((unsigned)joint<DEV_JOINT_COUNT) BSP_RobotBoard_MaskLimitInterrupt((uint8_t)joint);
}
void Dev_LimitSwitch_ResetDiagnosticInterrupts(void)
{
    BSP_RobotBoard_ResetLimitInterrupts();
}
bool Dev_LimitSwitch_TryRearmInterrupt(DevJointId_t joint)
{
    return (unsigned)joint<DEV_JOINT_COUNT && BSP_RobotBoard_TryRearmLimitInterrupt((uint8_t)joint);
}
