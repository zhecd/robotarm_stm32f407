#include "closed_loop.h"
#include "motor_core.h"
#include "bsp_stepper.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* ── PID 参数 ── */
#define CL_KP            0.25f   /* 比例增益 (降为 0.25 防超调) */
#define CL_KI            0.05f   /* 积分增益 */
#define CL_KD            0.15f   /* 微分增益 */
#define CL_INTEGRAL_MAX  5.0f    /* 积分限幅 (度) */
#define CL_OUTPUT_MAX    3.0f    /* 单周期最大输出 (度) ≈27 步 */
#define CL_DEADBAND_DEG  3.0f    /* 死区 (度) ≈27 步, 容忍机械回差 */
#define CL_UPDATE_MS     20U     /* PID 更新周期 (ms) = 50Hz */
#define CL_COOLDOWN_MS   500U    /* 校正冷却时间, 防止连续抖动 */

/* ── 补偿帧参数 ── */
#define CL_SPEED_DIV     50U
#define CL_MIN_TICKS     100U

MotorCL_t g_cl_m1;
MotorCL_t g_cl_m2;
MotorCL_t g_cl_m3;

static void PID_Init(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_max = CL_INTEGRAL_MAX;
    pid->output_max = CL_OUTPUT_MAX;
}

void CL_Init(void)
{
    g_cl_m1.pid = (PID_t){0};
    PID_Init(&g_cl_m1.pid, CL_KP, CL_KI, CL_KD);
    g_cl_m1.encoder = &Encoder_M1;
    g_cl_m1.target_deg = 0.0f;
    g_cl_m1.deadband_deg = CL_DEADBAND_DEG;
    g_cl_m1.enabled = true;
    g_cl_m1.last_correct_ms = 0;

    g_cl_m2.pid = (PID_t){0};
    PID_Init(&g_cl_m2.pid, CL_KP, CL_KI, CL_KD);
    g_cl_m2.encoder = &Encoder_M2;
    g_cl_m2.target_deg = 0.0f;
    g_cl_m2.deadband_deg = CL_DEADBAND_DEG;
    g_cl_m2.enabled = true;
    g_cl_m2.last_correct_ms = 0;

    g_cl_m3.pid = (PID_t){0};
    PID_Init(&g_cl_m3.pid, CL_KP, CL_KI, CL_KD);
    g_cl_m3.encoder = &Encoder_M3;
    g_cl_m3.target_deg = 0.0f;
    g_cl_m3.deadband_deg = CL_DEADBAND_DEG;
    g_cl_m3.enabled = true;
    g_cl_m3.last_correct_ms = 0;
}

void CL_SetTargetsFromTheory(void)
{
    int32_t t1, t2, t3;
    Motor_Core_GetTheorySteps(&t1, &t2, &t3);
    g_cl_m1.target_deg = (float)t1 * DEGREES_PER_STEP;
    g_cl_m2.target_deg = (float)t2 * DEGREES_PER_STEP;
    g_cl_m3.target_deg = (float)t3 * DEGREES_PER_STEP;

    /* 目标更新后重置积分, 避免旧积分干扰新目标 */
    g_cl_m1.pid.integral = 0.0f;
    g_cl_m2.pid.integral = 0.0f;
    g_cl_m3.pid.integral = 0.0f;
}

static float PID_Compute(PID_t *pid, float error, float dt)
{
    float abs_err = fabsf(error);

    /* P 项 */
    float p_out = pid->kp * error;

    /* I 项 — 积分分离: 大误差时不累积 */
    if (abs_err < 3.0f) {
        pid->integral += error * dt;
        if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
        if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    } else {
        pid->integral = 0.0f;
    }
    float i_out = pid->ki * pid->integral;

    /* D 项 */
    float d_out = (dt > 1e-6f) ? (pid->kd * (error - pid->prev_error) / dt) : 0.0f;
    pid->prev_error = error;

    float output = p_out + i_out + d_out;
    if (output >  pid->output_max) output =  pid->output_max;
    if (output < -pid->output_max) output = -pid->output_max;

    return output;
}

/*
 * 单轴误差计算, 返回应当补偿的微步数 (0 = 无需补偿)
 * 不直接推帧, 由上层合并三轴后再推一帧
 */
static int32_t CL_ComputeAxisSteps(MotorCL_t *cl, uint32_t now_ms)
{
    if (!cl->enabled || cl->encoder == NULL)
        return 0;

    /* 冷却检查: 距离上次校正必须超过 CL_COOLDOWN_MS */
    if (now_ms - cl->last_correct_ms < CL_COOLDOWN_MS)
        return 0;

    BSP_AS5600_Update(cl->encoder);

    float error = cl->target_deg - cl->encoder->angle_deg;
    while (error >  180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    if (fabsf(error) <= cl->deadband_deg) {
        cl->pid.integral = 0.0f;
        cl->pid.prev_error = 0.0f;
        return 0;
    }

    float correction_deg = PID_Compute(&cl->pid, error, (float)CL_UPDATE_MS * 0.001f);
    if (fabsf(correction_deg) < 0.01f)
        return 0;

    int32_t steps = (int32_t)roundf(correction_deg * STEPS_PER_DEGREE);
    return steps;
}

/*
 * 闭环主更新函数。
 * 将三轴补偿合并为一帧, 避免交替推帧导致的抖动。
 * 仅当运动缓冲区空闲时执行。
 */
void CL_Update(void)
{
    if (Motor_Core_IsRunning() || Motor_Buffer_GetCount() > 0)
        return;

    uint32_t now = HAL_GetTick();

    int32_t s1 = CL_ComputeAxisSteps(&g_cl_m1, now);
    int32_t s2 = CL_ComputeAxisSteps(&g_cl_m2, now);
    int32_t s3 = CL_ComputeAxisSteps(&g_cl_m3, now);

    if (s1 == 0 && s2 == 0 && s3 == 0)
        return;

    MotionFrame_t frame;
    frame.delta_m1 = s1;
    frame.delta_m2 = s2;
    frame.delta_m3 = s3;

    uint32_t max_delta = (uint32_t)labs(s1);
    if ((uint32_t)labs(s2) > max_delta) max_delta = (uint32_t)labs(s2);
    if ((uint32_t)labs(s3) > max_delta) max_delta = (uint32_t)labs(s3);
    frame.total_ticks = (max_delta * CL_SPEED_DIV > CL_MIN_TICKS)
                      ? (max_delta * CL_SPEED_DIV) : CL_MIN_TICKS;

    if (Motor_Buffer_Push(&frame)) {
        if (s1) g_cl_m1.last_correct_ms = now;
        if (s2) g_cl_m2.last_correct_ms = now;
        if (s3) g_cl_m3.last_correct_ms = now;
    }
}
