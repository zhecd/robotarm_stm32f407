#include "closed_loop.h"
#include "motor_core.h"
#include "common.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* ── PID 控制器 (模块私有) ── */
#define CL_KP            0.25f
#define CL_KI            0.05f
#define CL_KD            0.08f
#define CL_INTEGRAL_MAX  5.0f
#define CL_DEADBAND_DEG  5.0f
#define CL_UPDATE_MS     20U
#define CL_I_SEP_ERR     3.0f
#define CL_MIN_TICKS     100U
#define CL_SPEED_DIV     200U
#define CL_EMA_ALPHA     0.2f

/* 末端微调 (小误差): 慢速小幅, 防振荡 */
#define CL_OUTPUT_MAX_LO     1.0f
#define CL_COOLDOWN_LO_MS    800U

/* 丢步恢复 (大误差 > LARGE_ERR_DEG): 快速大力拉回 */
#define CL_LARGE_ERR_DEG     5.0f
#define CL_OUTPUT_MAX_HI     3.0f
#define CL_COOLDOWN_HI_MS    100U

typedef struct {
    float integral;
    float prev_error;
    float integral_max;
} PID_t;

static PID_t s_pid[CL_AXIS_COUNT];
static uint32_t s_last_correct_ms[CL_AXIS_COUNT];

MotorCL_t g_axis[CL_AXIS_COUNT];

/* ── PID 核心 ── */

static void PID_Reset(PID_t *p)
{
    p->integral = 0.0f;
    p->prev_error = 0.0f;
}

static float PID_Compute(MotorCL_t *cl, PID_t *p, float error, float dt,
                         float out_max)
{
    float out = cl->kp * error;

    if (fabsf(error) < CL_I_SEP_ERR) {
        p->integral += error * dt;
        if (p->integral >  p->integral_max) p->integral =  p->integral_max;
        if (p->integral < -p->integral_max) p->integral = -p->integral_max;
    } else {
        p->integral = 0.0f;
    }
    out += cl->ki * p->integral;

    if (dt > 1e-6f)
        out += cl->kd * (error - p->prev_error) / dt;
    p->prev_error = error;

    if (out >  out_max) out =  out_max;
    if (out < -out_max) out = -out_max;
    return out;
}

/* ── 公共 API ── */

void CL_Init(void)
{
    AS5600_t *enc[CL_AXIS_COUNT] = {&Encoder_M1, &Encoder_M2, &Encoder_M3};

    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        g_axis[i].kp        = CL_KP;
        g_axis[i].ki        = CL_KI;
        g_axis[i].kd        = CL_KD;
        g_axis[i].deadband_deg = CL_DEADBAND_DEG;
        g_axis[i].encoder   = enc[i];
        g_axis[i].target_deg = 0.0f;
        g_axis[i].enabled   = true;
        g_axis[i].filt_angle = 0.0f;
        g_axis[i].filt_valid = false;

        s_pid[i] = (PID_t){.integral_max = CL_INTEGRAL_MAX};
        s_last_correct_ms[i] = 0;
    }
}

void CL_SyncTarget(void)
{
    int32_t t[CL_AXIS_COUNT];
    Motor_Core_GetTheorySteps(&t[0], &t[1], &t[2]);
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        g_axis[i].target_deg = StepsToDeg(t[i]);
        g_axis[i].enabled = true;
        g_axis[i].filt_valid = false;   /* 复位滤波器, 下次读取时重新初始化 */
        PID_Reset(&s_pid[i]);
        s_last_correct_ms[i] = now;   /* 重置冷却, 先稳定再介入 */
    }
}

/* ── 单轴误差计算, 返回补偿步数 ── */
static int32_t ComputeCorrection(int i, uint32_t now_ms)
{
    MotorCL_t *cl = &g_axis[i];
    if (!cl->enabled || !cl->encoder) return 0;

    if (!BSP_AS5600_Update(cl->encoder)) return 0;  /* 读取失败, 不用假数据 */
    float raw = cl->encoder->angle_deg;
    if (!cl->filt_valid) {
        cl->filt_angle = raw;
        cl->filt_valid = true;
    } else {
        cl->filt_angle += CL_EMA_ALPHA * (raw - cl->filt_angle);
    }
    float error = AngleWrap180(cl->target_deg - cl->filt_angle);
    float abs_err = fabsf(error);

    if (abs_err <= cl->deadband_deg) {
        PID_Reset(&s_pid[i]);
        return 0;
    }

    /* 自适应: 大误差用快速强力参数, 小误差用保守参数防振荡 */
    bool large = (abs_err > CL_LARGE_ERR_DEG);
    uint32_t cooldown = large ? CL_COOLDOWN_HI_MS : CL_COOLDOWN_LO_MS;
    if (now_ms - s_last_correct_ms[i] < cooldown) return 0;

    float out_max = large ? CL_OUTPUT_MAX_HI : CL_OUTPUT_MAX_LO;
    float corr_deg = PID_Compute(cl, &s_pid[i], error,
                                 (float)CL_UPDATE_MS * 0.001f, out_max);
    if (fabsf(corr_deg) < 0.01f) return 0;

    return DegToSteps(corr_deg);
}

void CL_Update(void)
{
    if (Motor_Core_IsRunning() || Motor_Buffer_GetCount() > 0)
        return;

    uint32_t now = HAL_GetTick();
    int32_t s[CL_AXIS_COUNT];
    bool any = false;

    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        s[i] = ComputeCorrection(i, now);
        if (s[i]) any = true;
    }

    if (!any) return;

    MotionFrame_t frame = {
        .delta_m1 = s[0], .delta_m2 = s[1], .delta_m3 = s[2],
        .total_ticks = Common_MaxAbs3(s[0], s[1], s[2]) * CL_SPEED_DIV
    };
    if (frame.total_ticks < CL_MIN_TICKS)
        frame.total_ticks = CL_MIN_TICKS;

    if (Motor_Buffer_Push(&frame)) {
        for (int i = 0; i < CL_AXIS_COUNT; i++)
            if (s[i]) s_last_correct_ms[i] = now;
    }
}
