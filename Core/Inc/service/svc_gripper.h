/**
 * @file    ctrl_gripper.h
 * @brief   Gripper servo control interface / 澶圭埅鑸垫満鎺у埗鎺ュ�? * @ingroup control
 *
 * Thin abstraction over the BSP servo gripper. APP layer calls these
 * instead of Drv_Servo_* directly, preserving the APP鈫扖ONTROL鈫払SP
 * layer dependency chain.
 * BSP 鑸垫満鐨勮杽鎶借薄灞傘€侫PP 灞傝皟鐢ㄨ繖浜涙帴鍙ｈ€岄潪鐩存帴浣跨敤 Drv_Servo_*,
 * 缁存�?APP鈫扖ONTROL鈫払SP 鐨勫眰娆′緷璧栭摼銆? */

#ifndef __CTRL_GRIPPER_H__
#define __CTRL_GRIPPER_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void Svc_Gripper_Init(TIM_HandleTypeDef *htim);
void Svc_Gripper_Open(void);
void Svc_Gripper_Close(void);
void Svc_Gripper_IdleStop(void);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_GRIPPER_H__ */
