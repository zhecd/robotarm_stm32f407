/**
 * @file motion_service.c
 * @brief 运动公开入口：帧提交、执行状态、反馈补偿与一次性故障事件。
 */
#include "motion_service.h"

#include "safety_service.h"
#include "ctrl_motion_engine.h"
#include "ctrl_closed_loop.h"
#include "platform_critical.h"
#include "limit_irq_guard.h"

static volatile bool s_fault_event_pending;
static volatile bool s_fault_event_published;
static volatile MotionFaultReason_t s_fault_event_reason;

static void PublishFaultEventIfNeeded(void)
{
    if (!Ctrl_MotionEngine_HasFault()) return;

    PlatformCriticalState_t state = PlatformCritical_Enter();
    if (!s_fault_event_published) {
        s_fault_event_reason = Ctrl_MotionEngine_GetFaultReason();
        s_fault_event_pending = true;
        s_fault_event_published = true;
    }
    PlatformCritical_Exit(state);
}

void MotionService_Init(void)
{
    Ctrl_MotionEngine_Init();
    s_fault_event_pending = false;
    s_fault_event_published = false;
    s_fault_event_reason = MOTION_FAULT_NONE;
}

/** 主循环提交单帧；无运动许可、故障、队列满或非法帧均返回 false。 */
bool MotionService_SubmitFrame(const MotionFrame_t *frame)
{
    return LimitIrqGuard_GetLatchMask()==0U && SafetyService_IsMotionAllowed() && Ctrl_MotionEngine_PushFrame(frame);
}

void MotionService_OnStepTickFromISR(void)
{
    /* 与更高优先级限位IRQ互斥：过载停止后，旧tick不能继续拉高下一轴STEP。 */
    PlatformCriticalState_t key=PlatformCritical_Enter();
    if (LimitIrqGuard_GetLatchMask()==0U) Ctrl_MotionEngine_OnStepTickFromISR();
    PlatformCritical_Exit(key);
}

void MotionService_ServiceSafety(void)
{
    Ctrl_MotionEngine_ServiceSafety();
    PublishFaultEventIfNeeded();
}

bool MotionService_IsRunning(void)
{
    return Ctrl_MotionEngine_IsRunning();
}

bool MotionService_IsActionAllowed(void)
{
    MotionService_ServiceSafety();
    return SafetyService_IsMotionAllowed() && !Ctrl_MotionEngine_HasFault() &&
           LimitIrqGuard_GetLatchMask()==0U;
}

bool MotionService_IsIdle(void)
{
    /* 运行标志和队列长度属于同一快照，不能在两次读取间被 ISR 切换帧。 */
    PlatformCriticalState_t state=PlatformCritical_Enter();
    bool idle=!Ctrl_MotionEngine_IsRunning() && Ctrl_MotionEngine_GetQueueCount()==0U;
    PlatformCritical_Exit(state);
    return idle;
}

uint16_t MotionService_GetQueueCount(void)
{
    return Ctrl_MotionEngine_GetQueueCount();
}

/** 复制规划器累计的命令参考微步数，不等同于编码器实测值。 */
void MotionService_GetTheorySteps(int32_t *m1, int32_t *m2, int32_t *m3)
{
    Ctrl_MotionEngine_GetTheorySteps(m1, m2, m3);
}

void MotionService_AdjustTheorySteps(int32_t dm1, int32_t dm2, int32_t dm3)
{
    Ctrl_MotionEngine_AdjustTheorySteps(dm1, dm2, dm3);
}

void MotionService_ResetTheorySteps(void)
{
    Ctrl_MotionEngine_ResetTheorySteps();
}

/** 清除未来帧，不中断当前帧；立即停止应走 AbortForFault。 */
void MotionService_ClearQueuedFrames(void)
{
    Ctrl_MotionEngine_Clear();
}

void MotionService_AbortForFault(MotionFaultReason_t reason)
{
    Ctrl_MotionEngine_EmergencyStopWithReason(reason);
    PublishFaultEventIfNeeded();
}

void MotionService_SetLimitMonitoring(bool enabled)
{
    Ctrl_MotionEngine_EnableLimitMonitoring(enabled);
}

void MotionService_NotifyLimitSwitch(uint8_t joint_mask)
{
    Ctrl_MotionEngine_NotifyLimitSwitch(joint_mask);
}

MotionFaultReason_t MotionService_GetFaultReason(void)
{
    return Ctrl_MotionEngine_GetFaultReason();
}

bool MotionService_HasFault(void)
{
    return Ctrl_MotionEngine_HasFault();
}

/** 仅在重新回零和标定成功后调用，不能用于绕过故障恢复流程。 */
void MotionService_ClearFault(void)
{
    Ctrl_MotionEngine_ClearFault();
    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_fault_event_pending = false;
    s_fault_event_published = false;
    s_fault_event_reason = MOTION_FAULT_NONE;
    PlatformCritical_Exit(state);
}

/** 取走一次故障事件；每次故障锁存只发布 1 次，恢复后允许发布新事件。 */
bool MotionService_TakeFaultEvent(MotionFaultReason_t *out_reason)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    bool pending = s_fault_event_pending;
    if (pending) {
        if (out_reason) *out_reason = s_fault_event_reason;
        s_fault_event_pending = false;
    }
    PlatformCritical_Exit(state);
    return pending;
}

void MotionService_InitClosedLoop(void)
{
    Ctrl_ClosedLoop_Init();
}

void MotionService_SyncClosedLoopTarget(void)
{
    Ctrl_ClosedLoop_SyncTarget();
    PublishFaultEventIfNeeded();
}

void MotionService_UpdateClosedLoop(void)
{
    Ctrl_ClosedLoop_Update();
    PublishFaultEventIfNeeded();
}

bool MotionService_IsClosedLoopRecoveryActive(void)
{
    return Ctrl_ClosedLoop_IsRecoveryActive();
}

bool MotionService_HasClosedLoopSettled(void)
{
    return Ctrl_ClosedLoop_HasSettled();
}

void MotionService_GetClosedLoopStatus(ClosedLoopStatus_t *out)
{
    Ctrl_ClosedLoop_GetStatus(out);
}

bool MotionService_IsClosedLoopAxisEnabled(int axis)
{
    return Ctrl_ClosedLoop_IsAxisEnabled(axis);
}

void MotionService_SetClosedLoopAxisEnabled(int axis, bool enabled)
{
    Ctrl_ClosedLoop_SetAxisEnabled(axis, enabled);
}

bool MotionService_GetClosedLoopAxisAngle(int axis, float *out_deg)
{
    bool valid=Ctrl_ClosedLoop_GetAxisAngle(axis, out_deg);
    /* PS2只采样、不运行补偿，也必须交付本次读取发现的编码器/软限位故障。 */
    PublishFaultEventIfNeeded();
    return valid;
}

ErrorCode_t MotionService_SetClosedLoopAxisZero(int axis)
{
    return Ctrl_ClosedLoop_SetAxisZero(axis);
}
