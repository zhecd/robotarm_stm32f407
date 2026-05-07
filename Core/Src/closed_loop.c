#include "closed_loop.h"
#include "motor_core.h"
#include "common.h"
#include <math.h>
#include <stdlib.h>

/* ── PID 控制器 (模块私有) ── */
#define CL_KP            0.25f
#define CL_KI            0.05f
#define CL_KD            0.15f
#define CL_INTEGRAL_MAX  5.0f
#define CL_OUTPUT_MAX    3.0f
#define CL_DEADBAND_DEG  3.0f
#define CL_UPDATE_MS     20U
#define CL_COOLDOWN_MS   500U
#define CL_I_SEP_ERR     3.0f     /* 积分分离阈值 (度) */

#define CL_SPEED_DIV     50U
#define CL_MIN_TICKS     100U

typedef struct {
    float integral;
    float prev_error;
    float integral_max;
    float output_max;
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

static float PID_Compute(MotorCL_t *cl, PID_t *p, float error, float dt)
{
    /* P */
    float out = cl->kp * error;

    /* I (积分分离) */
    if (fabsf(error) < CL_I_SEP_ERR) {
        p->integral += error * dt;
        float lim = p->integral_max;
        if (p->integral >  lim) p->integral =  lim;
        if (p->integral < -lim) p->integral = -lim;
    } else {
        p->integral = 0.0f;
    }
    out += cl->ki * p->integral;

    /* D */
    if (dt > 1e-6f)
        out += cl->kd * (error - p->prev_error) / dt;
    p->prev_error = error;

    /* 输出限幅 */
    float lim = p->output_max;
    if (out >  lim) out =  lim;
    if (out < -lim) out = -lim;
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

        s_pid[i] = (PID_t){.integral_max = CL_INTEGRAL_MAX,
                           .output_max   = CL_OUTPUT_MAX};
        s_last_correct_ms[i] = 0;
    }
}

void CL_SyncTarget(void)
{
    int32_t t[CL_AXIS_COUNT];
    Motor_Core_GetTheorySteps(&t[0], &t[1], &t[2]);

    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        g_axis[i].target_deg = StepsToDeg(t[i]);
        PID_Reset(&s_pid[i]);
    }
}

/* ── 单轴误差计算, 返回补偿步数 ── */
static int32_t ComputeCorrection(int i, uint32_t now_ms)
{
    MotorCL_t *cl = &g_axis[i];
    if (!cl->enabled || !cl->encoder) return 0;
    if (now_ms - s_last_correct_ms[i] < CL_COOLDOWN_MS) return 0;

    BSP_AS5600_Update(cl->encoder);
    float error = AngleWrap180(cl->target_deg - cl->encoder->angle_deg);

    if (fabsf(error) <= cl->deadband_deg) {
        PID_Reset(&s_pid[i]);
        return 0;
    }

    float corr_deg = PID_Compute(cl, &s_pid[i], error,
                                 (float)CL_UPDATE_MS * 0.001f);
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
