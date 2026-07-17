#include "state_service.h"

#include "platform_critical.h"
#include "platform_time.h"

#include <string.h>

static StateServiceStatus_t s_status;

void StateService_Init(void)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    memset(&s_status, 0, sizeof(s_status));
    PlatformCritical_Exit(state);
}

void StateService_PublishAxisSample(uint8_t axis, float motor_angle_deg, float joint_angle_deg)
{
    if (axis >= ARM_AXIS_COUNT) return;
    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_status.motor_angle_deg[axis] = motor_angle_deg;
    s_status.joint_angle_deg[axis] = joint_angle_deg;
    s_status.encoder_valid[axis] = true;
    s_status.encoder_fail_count[axis] = 0U;
    s_status.timestamp_ms = PlatformTime_NowMs();
    s_status.generation++;
    PlatformCritical_Exit(state);
}

void StateService_PublishAxisReadFailure(uint8_t axis, uint8_t consecutive_failures)
{
    if (axis >= ARM_AXIS_COUNT) return;
    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_status.encoder_valid[axis] = false;
    s_status.encoder_fail_count[axis] = consecutive_failures;
    s_status.timestamp_ms = PlatformTime_NowMs();
    s_status.generation++;
    PlatformCritical_Exit(state);
}

void StateService_GetStatus(StateServiceStatus_t *out_status)
{
    if (!out_status) return;
    PlatformCriticalState_t state = PlatformCritical_Enter();
    *out_status = s_status;
    PlatformCritical_Exit(state);
}
