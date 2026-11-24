/** 使用真实SafetyService核对故障首因，避免将设备过载冒充主机M112。 */
#include "seek_result.h"
#include "safety_service.h"
#include <assert.h>
int main(void)
{
    SafetyService_Init();Seek_ReportTerminal(TRIAL_APP_FAILED,SEEK_IRQ);
    assert(SafetyService_GetFault()==SAFETY_FAULT_LIMIT_IRQ_OVERLOAD);
    Seek_ReportTerminal(TRIAL_HOST_STOP,SEEK_ABORT);
    assert(SafetyService_GetFault()==SAFETY_FAULT_LIMIT_IRQ_OVERLOAD);
    assert(!SafetyService_IsMotionAllowed());
    SafetyService_Init();Seek_ReportTerminal(TRIAL_COMPLETE,SEEK_OK);
    assert(!SafetyService_HasFault() && !SafetyService_IsMotionAllowed());
    SafetyServiceStatus_t s;SafetyService_GetStatus(&s);assert(!s.homed);
    SafetyService_Init();Seek_ReportTerminal(TRIAL_LEASE_LOST,SEEK_ABORT);
    assert(SafetyService_GetFault()==SAFETY_FAULT_INPUT_LOST);
    SafetyService_Init();Seek_ReportTerminal(TRIAL_APP_FAILED,SEEK_FEEDBACK);
    assert(SafetyService_GetFault()==SAFETY_FAULT_ENCODER);
    SafetyService_Init();Seek_ReportTerminal(TRIAL_ENCODER,SEEK_ABORT);
    assert(SafetyService_GetFault()==SAFETY_FAULT_ENCODER);
    SafetyService_Init();Seek_ReportTerminal(TRIAL_APP_FAILED,SEEK_UNSTABLE);
    assert(SafetyService_GetFault()==SAFETY_FAULT_INTERNAL);
    SafetyService_Init();Seek_ReportTerminal(TRIAL_HOST_STOP,SEEK_ABORT);
    assert(SafetyService_GetFault()==SAFETY_FAULT_HOST_STOP);
    Seek_ReportTerminal(TRIAL_APP_FAILED,SEEK_IRQ); /* 旧首因不被清理覆盖。 */
    assert(SafetyService_GetFault()==SAFETY_FAULT_HOST_STOP);
    return 0;
}
