/**
 * @file    ctrl_closed_loop.c
 * @brief   3-axis PID closed-loop position controller implementation. / 涓夎�?PID 闂幆浣嶇疆鎺у埗鍣ㄥ疄鐜般€?
 * @ingroup control
 */

#include "service/control/ctrl_closed_loop.h"
#include "service/control/ctrl_motion_engine.h"
#include "safety_service.h"
#include "state_service.h"
#include "device/dev_joint.h"
#include "robot_home_pose.h"
#include "robot_math.h"
#include "error_code.h"
#include <math.h>
#include <stdio.h>

/* 鈹€鈹€ Private PID state / 鍐呴�?PID 鐘舵�?鈹€鈹€ */
typedef struct {
    float integral;
    float prev_error;
    float integral_max;
} PID_State_t;

typedef struct {
    float          kp, ki, kd;
    float          deadband_deg;
    float          motor_angle_deg;
    float          target_deg;
    bool           enabled;
    float          filt_angle;
    bool           filt_valid;
    uint8_t        read_failures;
} AxisCL_t;

static AxisCL_t      s_axis[CL_AXIS_COUNT];
static PID_State_t   s_pid[CL_AXIS_COUNT];
static uint32_t      s_last_correct_ms[CL_AXIS_COUNT];

static bool UpdateEncoder(int axis)
{
    AxisCL_t *ax = &s_axis[axis];
    float motor_angle_deg = 0.0f;
    if (Dev_Joint_ReadMotorAngle((DevJointId_t)axis, &motor_angle_deg) == ERR_OK) {
#if JOINT_LIMITS_ENABLED
        if (!Dev_Joint_IsWithinSoftLimit((DevJointId_t)axis, motor_angle_deg)) {
            StateService_PublishAxisSample((uint8_t)axis, motor_angle_deg,
                RobotHomePose_MotorDegToJointDeg((RobotHomeAxis_t)axis, motor_angle_deg));
            SafetyService_ReportSoftLimit();
            return false;
        }
#endif
        ax->motor_angle_deg = motor_angle_deg;
        ax->read_failures = 0U;
        StateService_PublishAxisSample((uint8_t)axis, motor_angle_deg,
            RobotHomePose_MotorDegToJointDeg((RobotHomeAxis_t)axis, motor_angle_deg));
        return true;
    }
    ax->read_failures++;
    StateService_PublishAxisReadFailure((uint8_t)axis, ax->read_failures);
    if (ax->read_failures >= CL_ENCODER_FAIL_LIMIT)
        SafetyService_ReportEncoderFailure();
    return false;
}

/* 鈹€鈹€ PID core / PID 鏍稿�?鈹€鈹€ */

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

/* 鈹€鈹€ Per-axis correction / 鍗曡酱鏍℃ 鈹€鈹€ */

static int32_t ComputeCorrection(int i, uint32_t now_ms)
{
    AxisCL_t *ax = &s_axis[i];
    if (!ax->enabled) return 0;

    if (!UpdateEncoder(i)) return 0;

    float raw = ax->motor_angle_deg;
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

/* 鈹€鈹€ Public API / 鍏紑鎺ュ彛 鈹€鈹€ */

void Ctrl_ClosedLoop_Init(void)
{
    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        s_axis[i].kp          = CL_KP;
        s_axis[i].ki          = CL_KI;
        s_axis[i].kd          = CL_KD;
        s_axis[i].deadband_deg = CL_DEADBAND_DEG;
        s_axis[i].motor_angle_deg = 0.0f;
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

        UpdateEncoder(i);
    }
}

void Ctrl_ClosedLoop_Update(void)
{
    if (Ctrl_MotionEngine_HasFault()) return;
    uint32_t now = HAL_GetTick();
    bool     busy = Ctrl_MotionEngine_IsRunning() || Ctrl_MotionEngine_GetQueueCount() > 0;

    /* Always track multi-turn wraps, even when motion engine is busy.
       Otherwise a Nyquist gap opens during PID correction when the user
       is still forcing the motor. / 濮嬬粓杩借釜澶氬湀瓒婄�? 闃叉PID淇鏈熼棿
       鐢ㄦ埛鎸佺画鏆村姏鏃嬭浆瀵艰嚧鐨凬yquist閲囨牱缂哄彛�?*/
    if (busy) {
        for (int i = 0; i < CL_AXIS_COUNT; i++) {
            if (s_axis[i].enabled)
                UpdateEncoder(i);
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
        .total_ticks = RobotMath_MaxAbs3(s[0], s[1], s[2]) * CL_SPEED_DIV
    };
    if (frame.total_ticks < CL_MIN_TICKS)
        frame.total_ticks = CL_MIN_TICKS;

    if (Ctrl_MotionEngine_PushFrame(&frame)) {
        Ctrl_MotionEngine_AdjustTheorySteps(s[0], s[1], s[2]);
        for (int i = 0; i < CL_AXIS_COUNT; i++)
            if (s[i]) s_last_correct_ms[i] = now;
    }
}

/* 鈹€鈹€ Accessor functions / 璁块棶鍣ㄥ嚱�?鈹€鈹€ */

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
    if (axis < 0 || axis >= CL_AXIS_COUNT) {
        if (out_deg) *out_deg = 0.0f;
        return false;
    }
    if (!UpdateEncoder(axis)) {
        if (out_deg) *out_deg = 0.0f;
        return false;
    }
    if (out_deg) *out_deg = s_axis[axis].motor_angle_deg;
    return true;
}

ErrorCode_t Ctrl_ClosedLoop_SetAxisZero(int axis)
{
    if (axis < 0 || axis >= CL_AXIS_COUNT)
        return ERR_NULL_PARAM;
    return Dev_Joint_SetFeedbackZero((DevJointId_t)axis);
}
