#include "safety_service.h"

#include "platform_critical.h"
#include "motion_service.h"

static SafetyServiceStatus_t s_status;

static MotionFaultReason_t ToLegacyReason(SafetyFault_t fault)
{
    switch (fault) {
    case SAFETY_FAULT_LIMIT_SWITCH: return MOTION_FAULT_LIMIT_SWITCH;
    case SAFETY_FAULT_ENCODER: return MOTION_FAULT_ENCODER;
    case SAFETY_FAULT_SOFT_LIMIT: return MOTION_FAULT_SOFT_LIMIT;
    case SAFETY_FAULT_QUEUE_TIMEOUT: return MOTION_FAULT_QUEUE_TIMEOUT;
    default: return MOTION_FAULT_NONE;
    }
}

static SafetyFault_t FromLegacyReason(MotionFaultReason_t reason)
{
    switch (reason) {
    case MOTION_FAULT_LIMIT_SWITCH: return SAFETY_FAULT_LIMIT_SWITCH;
    case MOTION_FAULT_ENCODER: return SAFETY_FAULT_ENCODER;
    case MOTION_FAULT_SOFT_LIMIT: return SAFETY_FAULT_SOFT_LIMIT;
    case MOTION_FAULT_QUEUE_TIMEOUT: return SAFETY_FAULT_QUEUE_TIMEOUT;
    default: return SAFETY_FAULT_INTERNAL;
    }
}

static void LatchFault(SafetyFault_t fault)
{
    if (fault == SAFETY_FAULT_NONE) return;
    PlatformCriticalState_t state = PlatformCritical_Enter();
    bool already_latched = s_status.fault_latched;
    if (!already_latched) {
        s_status.fault_latched = true;
        s_status.motion_allowed = false;
        s_status.fault = fault;
        s_status.generation++;
    }
    PlatformCritical_Exit(state);

    if (!already_latched)
        MotionService_StopForSafety(ToLegacyReason(fault));
}

void SafetyService_Init(void)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_status = (SafetyServiceStatus_t){
        .generation = 1U,
        .motion_allowed = false,
        .homed = false,
        .fault_latched = false,
        .fault = SAFETY_FAULT_NONE,
    };
    PlatformCritical_Exit(state);
}

void SafetyService_ReportLimitSwitch(void) { LatchFault(SAFETY_FAULT_LIMIT_SWITCH); }
void SafetyService_ReportEncoderFailure(void) { LatchFault(SAFETY_FAULT_ENCODER); }
void SafetyService_ReportSoftLimit(void) { LatchFault(SAFETY_FAULT_SOFT_LIMIT); }
void SafetyService_ReportQueueTimeout(void) { LatchFault(SAFETY_FAULT_QUEUE_TIMEOUT); }

void SafetyService_ObserveLegacyMotionFault(void)
{
    if (MotionService_HasFault())
        LatchFault(FromLegacyReason(MotionService_GetFaultReason()));
}

bool SafetyService_IsMotionAllowed(void)
{
    SafetyServiceStatus_t snapshot;
    SafetyService_GetStatus(&snapshot);
    return snapshot.motion_allowed;
}

bool SafetyService_HasFault(void)
{
    SafetyServiceStatus_t snapshot;
    SafetyService_GetStatus(&snapshot);
    return snapshot.fault_latched;
}

SafetyFault_t SafetyService_GetFault(void)
{
    SafetyServiceStatus_t snapshot;
    SafetyService_GetStatus(&snapshot);
    return snapshot.fault;
}

void SafetyService_GetStatus(SafetyServiceStatus_t *out_status)
{
    if (!out_status) return;
    PlatformCriticalState_t state = PlatformCritical_Enter();
    *out_status = s_status;
    PlatformCritical_Exit(state);
}

void SafetyService_MarkHomed(void)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_status.homed = true;
    s_status.generation++;
    PlatformCritical_Exit(state);
}

void SafetyService_ClearAfterSuccessfulHoming(void)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    bool homed = s_status.homed;
    if (homed) {
        s_status.fault_latched = false;
        s_status.motion_allowed = true;
        s_status.fault = SAFETY_FAULT_NONE;
        s_status.generation++;
    }
    PlatformCritical_Exit(state);
    if (homed) MotionService_ClearFault();
}
