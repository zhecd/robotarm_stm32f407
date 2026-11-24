/**
 * @file state_service.h
 * @brief 状态服务：复制角度、有效性和时间戳快照，以 generation 标识更新。
 */
#ifndef ROBOTARM_STATE_SERVICE_H
#define ROBOTARM_STATE_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#define ARM_AXIS_COUNT 3U

typedef struct {
    uint32_t generation;
    uint32_t timestamp_ms; /* 最近一次任意轴发布事件时间，不代表所有轴的数据年龄。 */
    float motor_angle_deg[ARM_AXIS_COUNT];
    float joint_angle_deg[ARM_AXIS_COUNT];
    bool encoder_valid[ARM_AXIS_COUNT];
    uint8_t encoder_fail_count[ARM_AXIS_COUNT];
    bool sample_present[ARM_AXIS_COUNT]; /* 曾成功发布过样本；失败不覆盖最后成功时间。 */
    uint32_t sample_started_ms[ARM_AXIS_COUNT];
    uint32_t sample_finished_ms[ARM_AXIS_COUNT];
} StateServiceStatus_t;

void StateService_Init(void);
/** 发布单轴完整样本；motor_angle 为电机轴 deg，joint_angle 为关节 deg。 */
void StateService_PublishAxisSample(uint8_t axis, float motor_angle_deg, float joint_angle_deg);
/** 真实设备读取使用此入口；started在I2C之前取得，finished在返回后取得。
 * 非有限角度或反向时间戳记为读取失败，不发布可信的零角度。
 */
void StateService_PublishAxisSampleTimed(uint8_t axis,float motor_angle_deg,float joint_angle_deg,
                                       uint32_t started_ms,uint32_t finished_ms);
void StateService_PublishAxisReadFailure(uint8_t axis, uint8_t consecutive_failures);
/** 在已复制快照上计算逐轴新鲜度；max_age为排他上界，跨32位时钟回绕可用。
 * 失败/从未采样/年龄>=上界均无对应位，不改变运动许可或任何全局状态。
 */
uint8_t StateService_FreshMask(const StateServiceStatus_t *status,uint32_t now_ms,uint32_t max_age_ms);
/** 在短临界区复制快照；离开临界区后才能格式化日志或执行通信。 */
void StateService_GetStatus(StateServiceStatus_t *out_status);

#endif
