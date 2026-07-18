/**
 * @file dev_joint.h
 * @brief Robot-joint device abstraction.
 *
 * This module exposes joints rather than encoder or stepper-driver instances.
 */
#ifndef ROBOTARM_DEV_JOINT_H
#define ROBOTARM_DEV_JOINT_H

#include <stdbool.h>
#include <stdint.h>
#include "error_code.h"
#include "usart.h"

typedef enum {
    DEV_JOINT_M1 = 0,
    DEV_JOINT_M2,
    DEV_JOINT_M3,
    DEV_JOINT_COUNT
} DevJointId_t;

void Dev_Joint_Init(void);
void Dev_Joint_ConfigureDrivers(UART_HandleTypeDef *driver_uart);
void Dev_Joint_EnableAll(bool enable);
bool Dev_Joint_SetDirection(DevJointId_t joint, bool clockwise);
bool Dev_Joint_Step(DevJointId_t joint);

ErrorCode_t Dev_Joint_ReadMotorAngle(DevJointId_t joint, float *angle_deg);
ErrorCode_t Dev_Joint_SetFeedbackZero(DevJointId_t joint);
bool Dev_Joint_IsWithinSoftLimit(DevJointId_t joint, float motor_angle_deg);
void Dev_Joint_PrintFeedbackStatus(void);

#endif /* ROBOTARM_DEV_JOINT_H */
