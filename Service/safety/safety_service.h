#ifndef ROBOTARM_SAFETY_SERVICE_H
#define ROBOTARM_SAFETY_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SAFETY_FAULT_NONE = 0,
    SAFETY_FAULT_LIMIT_SWITCH,
    SAFETY_FAULT_ENCODER,
    SAFETY_FAULT_SOFT_LIMIT,
    SAFETY_FAULT_QUEUE_TIMEOUT,
    SAFETY_FAULT_INTERNAL
} SafetyFault_t;

typedef struct {
    uint32_t generation;
    bool motion_allowed;
    bool homed;
    bool fault_latched;
    SafetyFault_t fault;
} SafetyServiceStatus_t;

void SafetyService_Init(void);
void SafetyService_ReportLimitSwitch(void);
void SafetyService_ReportEncoderFailure(void);
void SafetyService_ReportSoftLimit(void);
void SafetyService_ReportQueueTimeout(void);
void SafetyService_ObserveLegacyMotionFault(void);
bool SafetyService_IsMotionAllowed(void);
bool SafetyService_HasFault(void);
SafetyFault_t SafetyService_GetFault(void);
void SafetyService_GetStatus(SafetyServiceStatus_t *out_status);
void SafetyService_MarkHomed(void);
void SafetyService_ClearAfterSuccessfulHoming(void);

#endif
