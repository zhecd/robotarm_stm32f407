#include <assert.h>
#include <stdio.h>

#include "platform_critical.h"
#include "safety_service.h"

PlatformCriticalState_t PlatformCritical_Enter(void)
{
    return 0U;
}

void PlatformCritical_Exit(PlatformCriticalState_t state)
{
    (void)state;
}

static void TestFaultLatchAndRecovery(void)
{
    SafetyService_Init();
    assert(!SafetyService_IsMotionAllowed());
    assert(!SafetyService_HasFault());

    SafetyService_ReportLimitSwitch();
    assert(SafetyService_HasFault());
    assert(SafetyService_GetFault() == SAFETY_FAULT_LIMIT_SWITCH);
    assert(!SafetyService_IsMotionAllowed());

    /* The first fault remains latched until homing has completed. */
    SafetyService_ReportEncoderFailure();
    assert(SafetyService_GetFault() == SAFETY_FAULT_LIMIT_SWITCH);
    SafetyService_ClearAfterSuccessfulHoming();
    assert(SafetyService_HasFault());

    SafetyService_MarkHomed();
    SafetyService_ClearAfterSuccessfulHoming();
    assert(!SafetyService_HasFault());
    assert(SafetyService_IsMotionAllowed());
}

int main(void)
{
    TestFaultLatchAndRecovery();
    puts("safety_service_tests: passed");
    return 0;
}
