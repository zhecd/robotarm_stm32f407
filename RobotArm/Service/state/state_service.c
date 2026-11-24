/**
 * @file state_service.c
 * @brief 状态服务：复制角度、有效性和时间戳快照，以 generation 标识更新。
 */
#include "state_service.h"

#include "platform_critical.h"
#include "platform_time.h"

#include <string.h>
#include <math.h>
#include <limits.h>

static StateServiceStatus_t s_status;

void StateService_Init(void)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    memset(&s_status, 0, sizeof(s_status));
    PlatformCritical_Exit(state);
}

/** 发布单轴完整样本；motor_angle 为电机轴 deg，joint_angle 为关节 deg。 */
void StateService_PublishAxisSample(uint8_t axis, float motor_angle_deg, float joint_angle_deg)
{
    /* 兼容已有发布者；真实I2C路径必须调用Timed，不能把返回时刻当成读取起点。 */
    uint32_t now=PlatformTime_NowMs();
    StateService_PublishAxisSampleTimed(axis,motor_angle_deg,joint_angle_deg,now,now);
}

void StateService_PublishAxisSampleTimed(uint8_t axis,float motor_angle_deg,float joint_angle_deg,
                                       uint32_t started_ms,uint32_t finished_ms)
{
    if (axis >= ARM_AXIS_COUNT) return;
    PlatformCriticalState_t state = PlatformCritical_Enter();
    if (!isfinite(motor_angle_deg) || !isfinite(joint_angle_deg) ||
        (uint32_t)(finished_ms-started_ms)>UINT32_MAX/2U) {
        s_status.encoder_valid[axis]=false;
        if(s_status.encoder_fail_count[axis]<UINT8_MAX)++s_status.encoder_fail_count[axis];
        s_status.timestamp_ms=PlatformTime_NowMs();s_status.generation++;
        PlatformCritical_Exit(state);return;
    }
    s_status.motor_angle_deg[axis] = motor_angle_deg;
    s_status.joint_angle_deg[axis] = joint_angle_deg;
    s_status.encoder_valid[axis] = true;
    s_status.encoder_fail_count[axis] = 0U;
    s_status.sample_present[axis]=true;
    s_status.sample_started_ms[axis]=started_ms;
    s_status.sample_finished_ms[axis]=finished_ms;
    s_status.timestamp_ms = PlatformTime_NowMs();
    s_status.generation++;
    PlatformCritical_Exit(state);
}

uint8_t StateService_FreshMask(const StateServiceStatus_t *status,uint32_t now,uint32_t max_age)
{
    if (!status || max_age==0U || max_age>UINT32_MAX/2U) return 0U;
    uint8_t mask=0U;
    for(uint8_t axis=0;axis<ARM_AXIS_COUNT;axis++) {
        if(status->sample_present[axis] && status->encoder_valid[axis] &&
           (uint32_t)(now-status->sample_started_ms[axis])<max_age)
            mask|=(uint8_t)(1U<<axis);
    }
    return mask;
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

/** 在短临界区复制快照；离开临界区后才能格式化日志或执行通信。 */
void StateService_GetStatus(StateServiceStatus_t *out_status)
{
    if (!out_status) return;
    PlatformCriticalState_t state = PlatformCritical_Enter();
    *out_status = s_status;
    PlatformCritical_Exit(state);
}
