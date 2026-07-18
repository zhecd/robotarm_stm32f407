#ifndef ROBOTARM_MOTION_SERVICE_H
#define ROBOTARM_MOTION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_types.h"
#include "common/error_code.h"

void MotionService_Init(void);
bool MotionService_SubmitFrame(const MotionFrame_t *frame);
void MotionService_OnStepTickFromISR(void);
void MotionService_ServiceSafety(void);
bool MotionService_IsRunning(void);
bool MotionService_IsIdle(void);
uint16_t MotionService_GetQueueCount(void);
void MotionService_GetTheorySteps(int32_t *m1, int32_t *m2, int32_t *m3);
void MotionService_AdjustTheorySteps(int32_t dm1, int32_t dm2, int32_t dm3);
void MotionService_ResetTheorySteps(void);
void MotionService_ClearQueuedFrames(void);
void MotionService_StopForSafety(MotionFaultReason_t reason);
void MotionService_SetLimitMonitoring(bool enabled);
void MotionService_NotifyLimitSwitch(uint16_t gpio_pin);
MotionFaultReason_t MotionService_GetFaultReason(void);
bool MotionService_HasFault(void);
void MotionService_ClearFault(void);

/* Closed-loop feedback is owned by the motion service; App must not access
 * its implementation directly. */
void MotionService_InitClosedLoop(void);
void MotionService_SyncClosedLoopTarget(void);
void MotionService_UpdateClosedLoop(void);
bool MotionService_IsClosedLoopRecoveryActive(void);
bool MotionService_IsClosedLoopAxisEnabled(int axis);
void MotionService_SetClosedLoopAxisEnabled(int axis, bool enabled);
bool MotionService_GetClosedLoopAxisAngle(int axis, float *out_deg);
ErrorCode_t MotionService_SetClosedLoopAxisZero(int axis);

#endif
