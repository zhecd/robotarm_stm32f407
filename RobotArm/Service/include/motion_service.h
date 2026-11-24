/**
 * @file motion_service.h
 * @brief 运动公开入口：帧提交、执行状态、反馈补偿与一次性故障事件。
 */
#ifndef ROBOTARM_MOTION_SERVICE_H
#define ROBOTARM_MOTION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_types.h"
#include "closed_loop_status.h"
#include "common/error_code.h"

void MotionService_Init(void);
/** 主循环提交单帧；无运动许可、故障、队列满或非法帧均返回 false。 */
bool MotionService_SubmitFrame(const MotionFrame_t *frame);
void MotionService_OnStepTickFromISR(void);
void MotionService_ServiceSafety(void);
/** 主循环动作入口：确认到期的限位，再综合本轮回零许可、Motion故障及IRQ锁存。
 * 不生成补偿、不做I2C；独立调用命令/夹爪/遥操作也须经过此门槛，禁止在ISR调用。
 */
bool MotionService_IsActionAllowed(void);
bool MotionService_IsRunning(void);
bool MotionService_IsIdle(void);
uint16_t MotionService_GetQueueCount(void);
/** 复制规划器累计的命令参考微步数，不等同于编码器实测值。 */
void MotionService_GetTheorySteps(int32_t *m1, int32_t *m2, int32_t *m3);
void MotionService_AdjustTheorySteps(int32_t dm1, int32_t dm2, int32_t dm3);
void MotionService_ResetTheorySteps(void);
/** 清除未来帧，不中断当前帧；立即停止应走 AbortForFault。 */
void MotionService_ClearQueuedFrames(void);
void MotionService_AbortForFault(MotionFaultReason_t reason);
void MotionService_SetLimitMonitoring(bool enabled);
/** ISR 使用逻辑轴位图通知限位；具体 GPIO 仅由 App ISR adapter 解释。 */
void MotionService_NotifyLimitSwitch(uint8_t joint_mask);
MotionFaultReason_t MotionService_GetFaultReason(void);
bool MotionService_HasFault(void);
/** 仅在重新回零和标定成功后调用，不能用于绕过故障恢复流程。 */
void MotionService_ClearFault(void);
/** 取走一次故障事件；每次故障锁存只发布 1 次，恢复后允许发布新事件。 */
bool MotionService_TakeFaultEvent(MotionFaultReason_t *out_reason);

/* 闭环反馈由运动服务拥有；App只访问公开合同，不能直接调用私有实现。 */
void MotionService_InitClosedLoop(void);
void MotionService_SyncClosedLoopTarget(void);
void MotionService_UpdateClosedLoop(void);
/** 主循环只复制闭环记录；不读I2C、不生成帧、不清故障；NULL无操作。 */
void MotionService_GetClosedLoopStatus(ClosedLoopStatus_t *out);
bool MotionService_IsClosedLoopRecoveryActive(void);
bool MotionService_HasClosedLoopSettled(void);
bool MotionService_IsClosedLoopAxisEnabled(int axis);
void MotionService_SetClosedLoopAxisEnabled(int axis, bool enabled);
bool MotionService_GetClosedLoopAxisAngle(int axis, float *out_deg);
ErrorCode_t MotionService_SetClosedLoopAxisZero(int axis);

#endif
