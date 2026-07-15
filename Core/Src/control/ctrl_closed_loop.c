/**
 * @file    ctrl_closed_loop.c
 * @brief   3-axis PID closed-loop position controller implementation. / 三轴 PID 闭环位置控制器实现。
 * @ingroup control
 */

#include "control/ctrl_closed_loop.h"
#include "control/ctrl_motion_engine.h"
#include "bsp/bsp_as5600.h"
#include "common.h"
#include "error_code.h"
#include <math.h>
#include <stdio.h>

/* ── Private PID state / 内部 PID 状态 ── */
typedef struct {
    float integral;
    float prev_error;
    float integral_max;
} PID_State_t;

typedef struct {
    float          kp, ki, kd;
    float          deadband_deg;
    AS5600_Dev_t  *encoder;
    float          target_deg;
    bool           enabled;
    float          filt_angle;
    bool           filt_valid;
    uint8_t        read_failures;
} AxisCL_t;

static AxisCL_t      s_axis[CL_AXIS_COUNT];
static PID_State_t   s_pid[CL_AXIS_COUNT];
static uint32_t      s_last_correct_ms[CL_AXIS_COUNT];

static bool UpdateEncoder(AxisCL_t *ax)
{
    if (BSP_AS5600_Update(ax->encoder) == ERR_OK) {
        ax->read_failures = 0U;
        return true;
    }
    if (++ax->read_failures >= CL_ENCODER_FAIL_LIMIT)
        Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_ENCODER);
    return false;
}

/* ── PID core / PID 核心 ── */

static void PID_Reset(PID_State_t *p)
{
    p->integral   = 0.0f;
    p->prev_error = 0.0f;
}

static float PID_Compute(AxisCL_t *cl, PID_State_t *p, float error, float dt,
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

/* ── Per-axis correction / 单轴校正 ── */

static int32_t ComputeCorrection(int i, uint32_t now_ms)
{
    AxisCL_t *ax = &s_axis[i];
    if (!ax->enabled || !ax->encoder) return 0;

    if (!UpdateEncoder(ax)) return 0;

    float raw = ax->encoder->angle_deg;
    if (!ax->filt_valid) {
        ax->filt_angle = raw;
        ax->filt_valid = true;
    } else {
        ax->filt_angle += CL_EMA_ALPHA * (raw - ax->filt_angle);
    }

    float error   = ax->target_deg - ax->filt_angle;
    float abs_err = fabsf(error);

    if (abs_err <= ax->deadband_deg) {
        PID_Reset(&s_pid[i]);
        return 0;
    }

    bool     large    = (abs_err > CL_LARGE_ERR_DEG);
    uint32_t cooldown = large ? CL_COOLDOWN_HI_MS : CL_COOLDOWN_LO_MS;
    if (now_ms - s_last_correct_ms[i] < cooldown) return 0;

    float out_max = large ? CL_OUTPUT_MAX_HI : CL_OUTPUT_MAX_LO;
    float corr_deg = PID_Compute(ax, &s_pid[i], error,
                                 (float)CL_UPDATE_MS * 0.001f, out_max);
    if (fabsf(corr_deg) < 0.01f) return 0;

    return DegToSteps(corr_deg);
}

/* ── Public API / 公开接口 ── */

void Ctrl_ClosedLoop_Init(void)
{
    AS5600_Dev_t *enc[CL_AXIS_COUNT] = {
        BSP_AS5600_GetM1(), BSP_AS5600_GetM2(), BSP_AS5600_GetM3()
    };

    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        s_axis[i].kp          = CL_KP;
        s_axis[i].ki          = CL_KI;
        s_axis[i].kd          = CL_KD;
        s_axis[i].deadband_deg = CL_DEADBAND_DEG;
        s_axis[i].encoder     = enc[i];
        s_axis[i].target_deg  = 0.0f;
        s_axis[i].enabled     = true;
        s_axis[i].filt_angle  = 0.0f;
        s_axis[i].filt_valid  = false;
        s_axis[i].read_failures = 0U;

        s_pid[i] = (PID_State_t){.integral_max = CL_INTEGRAL_MAX};
        s_last_correct_ms[i] = 0;
    }
}

void Ctrl_ClosedLoop_SyncTarget(void)
{
    int32_t t[CL_AXIS_COUNT];
    Ctrl_MotionEngine_GetTheorySteps(&t[0], &t[1], &t[2]);
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        s_axis[i].target_deg = StepsToDeg(t[i]);
        s_axis[i].enabled    = true;
        s_axis[i].filt_valid = false;
        PID_Reset(&s_pid[i]);
        s_last_correct_ms[i] = now;

        if (s_axis[i].encoder) {
            UpdateEncoder(&s_axis[i]);
            BSP_AS5600_SyncTurn(s_axis[i].encoder, s_axis[i].target_deg);
        }
    }
}

void Ctrl_ClosedLoop_Update(void)
{
    if (Ctrl_MotionEngine_HasFault()) return;
    uint32_t now = HAL_GetTick();
    bool     busy = Ctrl_MotionEngine_IsRunning() || Ctrl_MotionEngine_GetQueueCount() > 0;

    /* Always track multi-turn wraps, even when motion engine is busy.
       Otherwise a Nyquist gap opens during PID correction when the user
       is still forcing the motor. / 始终追踪多圈越界, 防止PID修正期间
       用户持续暴力旋转导致的Nyquist采样缺口。 */
    if (busy) {
        for (int i = 0; i < CL_AXIS_COUNT; i++) {
            if (s_axis[i].enabled && s_axis[i].encoder)
                UpdateEncoder(&s_axis[i]);
        }
        return;
    }

    int32_t  s[CL_AXIS_COUNT];
    bool     any = false;

    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        s[i] = ComputeCorrection(i, now);
        if (s[i]) any = true;
    }

    if (!any) return;

    MotionFrame_t frame = {
        .delta_m1    = s[0],
        .delta_m2    = s[1],
        .delta_m3    = s[2],
        .total_ticks = Common_MaxAbs3(s[0], s[1], s[2]) * CL_SPEED_DIV
    };
    if (frame.total_ticks < CL_MIN_TICKS)
        frame.total_ticks = CL_MIN_TICKS;

    if (Ctrl_MotionEngine_PushFrame(&frame)) {
        Ctrl_MotionEngine_AdjustTheorySteps(s[0], s[1], s[2]);
        for (int i = 0; i < CL_AXIS_COUNT; i++)
            if (s[i]) s_last_correct_ms[i] = now;
    }
}

/* ── Accessor functions / 访问器函数 ── */

bool Ctrl_ClosedLoop_IsAxisEnabled(int axis)
{
    return (axis >= 0 && axis < CL_AXIS_COUNT) ? s_axis[axis].enabled : false;
}

void Ctrl_ClosedLoop_SetAxisEnabled(int axis, bool en)
{
    if (axis >= 0 && axis < CL_AXIS_COUNT)
        s_axis[axis].enabled = en;
}

bool Ctrl_ClosedLoop_GetAxisAngle(int axis, float *out_deg)
{
    if (axis < 0 || axis >= CL_AXIS_COUNT || !s_axis[axis].encoder) {
        if (out_deg) *out_deg = 0.0f;
        return false;
    }
    if (!UpdateEncoder(&s_axis[axis])) {
        if (out_deg) *out_deg = 0.0f;
        return false;
    }
    if (out_deg) *out_deg = s_axis[axis].encoder->angle_deg;
    return true;
}

ErrorCode_t Ctrl_ClosedLoop_SetAxisZero(int axis)
{
    if (axis < 0 || axis >= CL_AXIS_COUNT || !s_axis[axis].encoder)
        return ERR_NULL_PARAM;
    return BSP_AS5600_SetZero(s_axis[axis].encoder);
}
