#include "motion_service.h"

#include "safety_service.h"
#include "ctrl_motion_engine.h"
#include "ctrl_closed_loop.h"
#include "platform_critical.h"

static volatile bool s_fault_event_pending;
static volatile MotionFaultReason_t s_fault_event_reason;

static void PublishFaultEventIfNeeded(void)
{
    if (!Ctrl_MotionEngine_HasFault()) return;

    PlatformCriticalState_t state = PlatformCritical_Enter();
    if (!s_fault_event_pending) {
        s_fault_event_reason = Ctrl_MotionEngine_GetFaultReason();
        s_fault_event_pending = true;
    }
    PlatformCritical_Exit(state);
}

void MotionService_Init(void)
{
    Ctrl_MotionEngine_Init();
    s_fault_event_pending = false;
    s_fault_event_reason = MOTION_FAULT_NONE;
}

bool MotionService_SubmitFrame(const MotionFrame_t *frame)
{
    return SafetyService_IsMotionAllowed() && Ctrl_MotionEngine_PushFrame(frame);
}

void MotionService_OnStepTickFromISR(void)
{
    Ctrl_MotionEngine_OnStepTickFromISR();
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

bool MotionService_IsIdle(void)
{
    return !Ctrl_MotionEngine_IsRunning() && Ctrl_MotionEngine_GetQueueCount() == 0U;
}

uint16_t MotionService_GetQueueCount(void)
{
    return Ctrl_MotionEngine_GetQueueCount();
}

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

void MotionService_NotifyLimitSwitch(uint16_t gpio_pin)
{
    Ctrl_MotionEngine_NotifyLimitSwitch(gpio_pin);
}

MotionFaultReason_t MotionService_GetFaultReason(void)
{
    return Ctrl_MotionEngine_GetFaultReason();
}

bool MotionService_HasFault(void)
{
    return Ctrl_MotionEngine_HasFault();
}

void MotionService_ClearFault(void)
{
    Ctrl_MotionEngine_ClearFault();
    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_fault_event_pending = false;
    s_fault_event_reason = MOTION_FAULT_NONE;
    PlatformCritical_Exit(state);
}

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
    return Ctrl_ClosedLoop_GetAxisAngle(axis, out_deg);
}

ErrorCode_t MotionService_SetClosedLoopAxisZero(int axis)
{
    return Ctrl_ClosedLoop_SetAxisZero(axis);
}
