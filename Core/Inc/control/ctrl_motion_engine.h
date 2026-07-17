/**
 * @file    ctrl_motion_engine.h
 * @brief   Bresenham-based stepper motion engine with lock-free ring buffer / 基于 Bresenham 算法的步进运动引擎 + 无锁环形缓冲
 * @ingroup control
 *
 * Driven by TIM6 at 50 kHz. Each MotionFrame_t specifies delta steps for
 * M1/M2/M3 spread over total_ticks interrupts. Frames are queued via a
 * 64-entry lock-free ring buffer.
 * 由 TIM6 50kHz 驱动。每帧指定 M1/M2/M3 的增量步数, 分散到 total_ticks 次中断中执行。
 * 帧通过 64 项无锁环形缓冲区排队。
 *
 * Theory-step tracking provides the "commanded position" reference for
 * the closed-loop PID controller.
 * 理论步数追踪为 PID 闭环控制器提供"命令位置"参考。
 */

#ifndef __CTRL_MOTION_ENGINE_H__
#define __CTRL_MOTION_ENGINE_H__

#include <stdint.h>
#include <stdbool.h>
#include "bsp/bsp_stepper.h"

#ifdef __cplusplus
extern "C" {
#endif

/** A single motion segment: delta steps for 3 axes + duration in timer ticks / 单段运动: 三轴增量步数 + 定时器节拍时长 */
typedef struct {
    int32_t  delta_m1;          /* M1 axis step delta / M1 轴步数增量 */
    int32_t  delta_m2;          /* M2 axis step delta / M2 轴步数增量 */
    int32_t  delta_m3;          /* M3 axis step delta / M3 轴步数增量 */
    uint32_t total_ticks;       /* Duration in TIM6 ticks / TIM6 节拍时长 */
} MotionFrame_t;

typedef enum {
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_LIMIT_SWITCH,
    MOTION_FAULT_ENCODER,
    MOTION_FAULT_SOFT_LIMIT,
    MOTION_FAULT_QUEUE_TIMEOUT
} MotionFaultReason_t;

void Ctrl_MotionEngine_Init(void);

/** Push a frame into the ring buffer. Returns false if full. / 压入一帧到环形缓冲, 满时返回 false */
bool Ctrl_MotionEngine_PushFrame(const MotionFrame_t *frame);

/** Clear all queued frames (emergency stop) / 清空所有排队帧 (急停) */
void Ctrl_MotionEngine_Clear(void);

/* Stops the active frame and discards queued frames.  A fault remains latched
 * until it is explicitly cleared after a successful homing sequence. */
void Ctrl_MotionEngine_EmergencyStop(void);
void Ctrl_MotionEngine_EmergencyStopWithReason(MotionFaultReason_t reason);
void Ctrl_MotionEngine_EnableLimitMonitoring(bool enabled);
void Ctrl_MotionEngine_NotifyLimitSwitch(uint16_t gpio_pin);
void Ctrl_MotionEngine_ServiceSafety(void);
bool Ctrl_MotionEngine_HasFault(void);
MotionFaultReason_t Ctrl_MotionEngine_GetFaultReason(void);
void Ctrl_MotionEngine_ClearFault(void);

bool     Ctrl_MotionEngine_IsRunning(void);       /* Motion in progress / 运动中 */
uint16_t Ctrl_MotionEngine_GetQueueCount(void);   /* Queued frame count / 排队帧数 */

/** Read the accumulated "theory" (commanded) step positions / 读取累积的理论(命令)步数位置 */
void Ctrl_MotionEngine_GetTheorySteps(int32_t *m1, int32_t *m2, int32_t *m3);
void Ctrl_MotionEngine_ResetTheorySteps(void);
void Ctrl_MotionEngine_AdjustTheorySteps(int32_t dm1, int32_t dm2, int32_t dm3);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_MOTION_ENGINE_H__ */
