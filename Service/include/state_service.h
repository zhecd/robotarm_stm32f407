#ifndef ROBOTARM_STATE_SERVICE_H
#define ROBOTARM_STATE_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#define ARM_AXIS_COUNT 3U

typedef struct {
    uint32_t generation;
    uint32_t timestamp_ms;
    float motor_angle_deg[ARM_AXIS_COUNT];
    float joint_angle_deg[ARM_AXIS_COUNT];
    bool encoder_valid[ARM_AXIS_COUNT];
    uint8_t encoder_fail_count[ARM_AXIS_COUNT];
} StateServiceStatus_t;

void StateService_Init(void);
void StateService_PublishAxisSample(uint8_t axis, float motor_angle_deg, float joint_angle_deg);
void StateService_PublishAxisReadFailure(uint8_t axis, uint8_t consecutive_failures);
void StateService_GetStatus(StateServiceStatus_t *out_status);

#endif
