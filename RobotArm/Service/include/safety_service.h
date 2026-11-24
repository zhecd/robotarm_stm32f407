/**
 * @file safety_service.h
 * @brief 安全服务：锁存首个故障，维护回零状态与运动许可，不直接操作执行器。
 */
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
    SAFETY_FAULT_CONTROL_DIVERGENCE,
    SAFETY_FAULT_INTERNAL,
    SAFETY_FAULT_HOST_STOP,
    SAFETY_FAULT_LIMIT_IRQ_OVERLOAD,
    SAFETY_FAULT_INPUT_LOST,
    SAFETY_FAULT_DRIVER_CONFIG /* 10：启动配置未确认；保留原协议0..9的数值。 */
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
void SafetyService_ReportControlDivergence(void);
void SafetyService_ReportHostStop(void);
void SafetyService_ReportInternalFailure(void);
void SafetyService_ReportLimitIrqOverload(void);
void SafetyService_ReportInputLost(void);
/** 通信/配置确认失败，不据此判定驱动硬件损坏或供电故障。 */
void SafetyService_ReportDriverConfigFailure(void);
bool SafetyService_IsMotionAllowed(void);
bool SafetyService_HasFault(void);
SafetyFault_t SafetyService_GetFault(void);
void SafetyService_GetStatus(SafetyServiceStatus_t *out_status);
/** 只有本轮回零和编码器标定成功后才可声明已建立零位参考。 */
void SafetyService_MarkHomed(void);
/** 只有当前 homed 有效才清除锁存并开放运动；故障会使旧 homed 失效。 */
void SafetyService_ClearAfterSuccessfulHoming(void);
/** 试验结束撤销参考/许可，不制造主机停止故障，也绝不清除已有故障。 */
void SafetyService_RevokePermission(void);

#endif
