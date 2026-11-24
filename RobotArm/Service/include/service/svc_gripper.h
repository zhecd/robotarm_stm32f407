/**
 * @file svc_gripper.h
 * @brief 夹爪服务：表达打开、关闭和保持结束后的停 PWM 意图。
 */

#ifndef __CTRL_GRIPPER_H__
#define __CTRL_GRIPPER_H__
#include <stdbool.h>
#include "gripper_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 主循环：仅本轮回零许可有效、Motion无故障且IRQ未锁存时执行；拒绝返回false。
 * true表示底层已受理目标或同目标正在保持；PWM启动失败也返回false，
 * 现有命令服务将返回error: command rejected (-8)，不伪造M3OK/M5OK。
 * 不是机械到位确认。启动夹爪初始化仍由App另行编排，保持原功能。
 */
bool Svc_Gripper_Open(void);
bool Svc_Gripper_Close(void);
void Svc_Gripper_IdleStop(void);
/** 查询不检查/改变动作许可，不调用初始化、IdleStop或任何HAL写操作。 */
void Svc_Gripper_GetStatus(GripperStatus_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_GRIPPER_H__ */
