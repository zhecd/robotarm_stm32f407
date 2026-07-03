/**
 * @file    ctrl_gripper.h
 * @brief   Gripper servo control interface / 夹爪舵机控制接口
 * @ingroup control
 *
 * Thin abstraction over the BSP servo gripper. APP layer calls these
 * instead of BSP_Gripper_* directly, preserving the APP→CONTROL→BSP
 * layer dependency chain.
 * BSP 舵机的薄抽象层。APP 层调用这些接口而非直接使用 BSP_Gripper_*,
 * 维持 APP→CONTROL→BSP 的层次依赖链。
 */

#ifndef __CTRL_GRIPPER_H__
#define __CTRL_GRIPPER_H__

#ifdef __cplusplus
extern "C" {
#endif

void Ctrl_Gripper_Init(void);
void Ctrl_Gripper_Open(void);
void Ctrl_Gripper_Close(void);
void Ctrl_Gripper_IdleStop(void);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_GRIPPER_H__ */
