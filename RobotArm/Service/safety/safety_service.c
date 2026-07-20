#include "safety_service.h"

#include "platform_critical.h"

static SafetyServiceStatus_t s_status;

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
void SafetyService_ReportControlDivergence(void) { LatchFault(SAFETY_FAULT_CONTROL_DIVERGENCE); }

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
}
