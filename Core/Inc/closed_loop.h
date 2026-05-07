#ifndef __CLOSED_LOOP_H__
#define __CLOSED_LOOP_H__

#include <stdbool.h>
#include <stdint.h>
#include "bsp_as5600.h"

/* PID 控制器 */
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float integral_max;   /* 积分限幅 (度) */
    float output_max;     /* 输出限幅 (度/周期) */
} PID_t;

/* 单轴闭环控制器 */
typedef struct {
    PID_t pid;
    AS5600_t *encoder;
    float target_deg;     /* 目标角度 (0~360) */
    float deadband_deg;   /* 死区 (度) */
    bool enabled;
    uint32_t last_correct_ms;
} MotorCL_t;

extern MotorCL_t g_cl_m1;
extern MotorCL_t g_cl_m2;
extern MotorCL_t g_cl_m3;

void CL_Init(void);
void CL_SetTargetsFromTheory(void);
void CL_Update(void);  /* 主循环中以~50Hz 调用 */

#endif
