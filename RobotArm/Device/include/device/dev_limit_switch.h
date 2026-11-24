/**
 * @file dev_limit_switch.h
 * @brief 限位设备：以逻辑轴查询开关，GPIO 端口和有效电平由 BSP 解释。
 */
#ifndef ROBOTARM_DEV_LIMIT_SWITCH_H
#define ROBOTARM_DEV_LIMIT_SWITCH_H

#include <stdbool.h>
#include <stdint.h>
#include "device/dev_joint.h"

bool Dev_LimitSwitch_IsTriggered(DevJointId_t joint);
/** 仅只读诊断限流使用；屏蔽IRQ仍可正常读取该轴电平。 */
void Dev_LimitSwitch_MaskDiagnosticInterrupt(DevJointId_t joint);
/** 诊断初始化临界区中调用；不是运动故障恢复接口。 */
void Dev_LimitSwitch_ResetDiagnosticInterrupts(void);
/** 逻辑轴单路屏蔽，供过载停止与诊断策略使用；不影响GPIO读取。 */
void Dev_LimitSwitch_MaskInterrupt(DevJointId_t joint);
/** 调用者持短临界区且已检查无过载锁存；只恢复稳定释放后的这一轴。 */
bool Dev_LimitSwitch_TryRearmInterrupt(DevJointId_t joint);

#endif /* ROBOTARM_DEV_LIMIT_SWITCH_H */
