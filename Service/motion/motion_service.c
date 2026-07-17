#include "motion_service.h"

#include "safety_service.h"

void MotionService_Init(void)
{
    Ctrl_MotionEngine_Init();
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
    SafetyService_ObserveLegacyMotionFault();
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

void MotionService_StopForSafety(MotionFaultReason_t reason)
{
    Ctrl_MotionEngine_EmergencyStopWithReason(reason);
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
