#include "seek_result.h"
#include "safety_service.h"
void Seek_ReportTerminal(TrialReason t, SeekReason s)
{
    if(t==TRIAL_COMPLETE)return; /* 当前从未MarkHomed，因此不会开放动作许可。 */
    if(s==SEEK_IRQ)SafetyService_ReportLimitIrqOverload();
    else if(t==TRIAL_ENCODER || s==SEEK_FEEDBACK)SafetyService_ReportEncoderFailure();
    else if(t==TRIAL_HOST_STOP)SafetyService_ReportHostStop();
    else if(t==TRIAL_LEASE_LOST)SafetyService_ReportInputLost();
    else SafetyService_ReportInternalFailure();
}
