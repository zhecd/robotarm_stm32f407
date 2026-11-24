/**
 * @file ctrl_motion_engine.h
 * @brief 运动内部实现：单生产者/单消费者帧队列与定时整数插补。
 * @details 主循环生产帧，定时 ISR 消费；故障锁存和限位消抖暂停均在输出 STEP 前检查。
 */

#ifndef __CTRL_MOTION_ENGINE_H__
#define __CTRL_MOTION_ENGINE_H__

#include <stdint.h>
#include <stdbool.h>
#include "motion_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 帧数据定义在公开的 motion_types.h，此处只声明私有执行接口。 */
void Ctrl_MotionEngine_Init(void);

/* Push a frame into the ring buffer. Returns false if full. */
bool Ctrl_MotionEngine_PushFrame(const MotionFrame_t *frame);

/* 清除排队帧，保留当前帧；立即停止使用 EmergencyStopWithReason。 */
void Ctrl_MotionEngine_Clear(void);

/* Stops the active frame and discards queued frames.  A fault remains latched
 * until it is explicitly cleared after a successful homing sequence. */
void Ctrl_MotionEngine_EmergencyStop(void);
void Ctrl_MotionEngine_EmergencyStopWithReason(MotionFaultReason_t reason);
void Ctrl_MotionEngine_EnableLimitMonitoring(bool enabled);
void Ctrl_MotionEngine_NotifyLimitSwitch(uint8_t joint_mask);
void Ctrl_MotionEngine_ServiceSafety(void);
bool Ctrl_MotionEngine_HasFault(void);
MotionFaultReason_t Ctrl_MotionEngine_GetFaultReason(void);
void Ctrl_MotionEngine_ClearFault(void);

bool     Ctrl_MotionEngine_IsRunning(void);       /* 当前运动帧正在执行。 */
uint16_t Ctrl_MotionEngine_GetQueueCount(void);   /* Queued frame count */

/* Read the accumulated "theory" (commanded) step positions */
void Ctrl_MotionEngine_GetTheorySteps(int32_t *m1, int32_t *m2, int32_t *m3);
void Ctrl_MotionEngine_ResetTheorySteps(void);
void Ctrl_MotionEngine_AdjustTheorySteps(int32_t dm1, int32_t dm2, int32_t dm3);

/** Execute one 50 kHz step tick.  Only the App ISR adapter may call this. */
/** 每次调用执行一个定时节拍；时间上限与 STEP 脉宽需后续实机测量。 */
void Ctrl_MotionEngine_OnStepTickFromISR(void);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_MOTION_ENGINE_H__ */
