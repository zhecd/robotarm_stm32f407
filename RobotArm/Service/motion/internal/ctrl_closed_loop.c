/**
 * @file    ctrl_closed_loop.c
 * @brief   3-axis PID closed-loop position controller implementation. / 涓夎�?PID 闂幆浣嶇疆鎺у埗鍣ㄥ疄鐜般€?
 * @ingroup control
 */

#include "ctrl_closed_loop.h"
#include "ctrl_motion_engine.h"
#include "safety_service.h"
#include "state_service.h"
#include "device/dev_joint.h"
#include "platform_time.h"
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
    float          pending_error_abs;
    uint8_t        divergence_count;
    uint8_t        no_progress_count;
    bool           correction_pending;
    bool           pending_is_recovery;
} AxisCL_t;

typedef struct {
    bool     active;
    int32_t  total[CL_AXIS_COUNT];
    int32_t  generated[CL_AXIS_COUNT];
    uint32_t total_ticks;
    uint32_t segments;
    uint32_t next_segment;
    uint32_t previous_ticks;
} RecoveryTrajectory_t;

static AxisCL_t      s_axis[CL_AXIS_COUNT];
static PID_State_t   s_pid[CL_AXIS_COUNT];
static uint32_t      s_last_correct_ms[CL_AXIS_COUNT];
static bool          s_was_busy;
static uint32_t      s_settle_until_ms;
static RecoveryTrajectory_t s_recovery;

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

static float Smoothstep(float progress)
{
    if (progress <= 0.0f) return 0.0f;
    if (progress >= 1.0f) return 1.0f;

    /* Quintic profile: position, velocity and acceleration are continuous
       at both ends.  This gives autonomous recovery the same gentle
       start/stop characteristic expected from a normal planned move. */
    return progress * progress * progress *
           (progress * (progress * 6.0f - 15.0f) + 10.0f);
}

static bool RecoveryTrajectory_Fill(void)
{
    while (s_recovery.active &&
           Ctrl_MotionEngine_GetQueueCount() < CL_RECOVERY_QUEUE_AHEAD &&
           s_recovery.next_segment <= s_recovery.segments) {
        uint32_t index = s_recovery.next_segment++;
        uint32_t cumulative_ticks = (s_recovery.total_ticks * index) /
                                    s_recovery.segments;
        uint32_t frame_ticks = cumulative_ticks - s_recovery.previous_ticks;
        s_recovery.previous_ticks = cumulative_ticks;
        if (frame_ticks < CL_MIN_TICKS)
            frame_ticks = CL_MIN_TICKS;

        float progress = Smoothstep((float)index / (float)s_recovery.segments);
        int32_t target[CL_AXIS_COUNT] = {
            (int32_t)lroundf((float)s_recovery.total[0] * progress),
            (int32_t)lroundf((float)s_recovery.total[1] * progress),
            (int32_t)lroundf((float)s_recovery.total[2] * progress)
        };
        MotionFrame_t frame = {
            .delta_m1 = target[0] - s_recovery.generated[0],
            .delta_m2 = target[1] - s_recovery.generated[1],
            .delta_m3 = target[2] - s_recovery.generated[2],
            .total_ticks = frame_ticks
        };
        s_recovery.generated[0] = target[0];
        s_recovery.generated[1] = target[1];
        s_recovery.generated[2] = target[2];

        /* Zero-step frames intentionally preserve the low-speed parts at
           the beginning and end of the profile; dropping them would create
           a velocity jump. */
        if (!Ctrl_MotionEngine_PushFrame(&frame)) {
            Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_QUEUE_TIMEOUT);
            s_recovery.active = false;
            return false;
        }
    }
    return true;
}

static bool RecoveryTrajectory_Start(const int32_t correction_steps[CL_AXIS_COUNT])
{
    uint32_t max_steps = RobotMath_MaxAbs3(correction_steps[0],
                                            correction_steps[1],
                                            correction_steps[2]);
    if (max_steps == 0U || s_recovery.active) return false;

    uint32_t segments = (max_steps + CL_RECOVERY_STEPS_PER_SEGMENT - 1U) /
                        CL_RECOVERY_STEPS_PER_SEGMENT;
    if (segments < 2U) segments = 2U;

    s_recovery = (RecoveryTrajectory_t){
        .active = true,
        .total = {correction_steps[0], correction_steps[1], correction_steps[2]},
        .total_ticks = max_steps * CL_RECOVERY_TICKS_PER_STEP,
        .segments = segments,
        .next_segment = 1U,
        .previous_ticks = 0U,
    };
    if (s_recovery.total_ticks < CL_MIN_TICKS)
        s_recovery.total_ticks = CL_MIN_TICKS;

    return RecoveryTrajectory_Fill();
}

/* 鈹€鈹€ Per-axis correction / 鍗曡酱鏍℃ 鈹€鈹€ */

static int32_t ComputeCorrection(int i, uint32_t now_ms)
{
    AxisCL_t *ax = &s_axis[i];
    if (!ax->enabled) return 0;

    if (!UpdateEncoder(i)) return 0;

    /* Both values are in motor-shaft degrees.  The planner converts joint
       motion to motor steps with UNITS_PER_DEGREE; StepsToDeg() converts
       those steps back with the motor-shaft step resolution. */
    float raw = ax->motor_angle_deg;
    if (!ax->filt_valid) {
        ax->filt_angle = raw;
        ax->filt_valid = true;
    } else {
        ax->filt_angle += CL_EMA_ALPHA * (raw - ax->filt_angle);
    }

    float error   = ax->target_deg - ax->filt_angle;
    float abs_err = fabsf(error);

    if (ax->correction_pending) {
        if (abs_err > ax->pending_error_abs + CL_DIVERGENCE_TOLERANCE_DEG) {
            if (++ax->divergence_count >= CL_DIVERGENCE_LIMIT) {
                SafetyService_ReportControlDivergence();
                return 0;
            }
        } else {
            ax->divergence_count = 0U;
            if (ax->pending_is_recovery &&
                (ax->pending_error_abs - abs_err) < CL_RECOVERY_MIN_PROGRESS_DEG) {
                if (++ax->no_progress_count >= CL_RECOVERY_NO_PROGRESS_LIMIT) {
                    SafetyService_ReportControlDivergence();
                    return 0;
                }
            } else {
                ax->no_progress_count = 0U;
            }
        }
        ax->correction_pending = false;
        ax->pending_is_recovery = false;
    }

    if (abs_err <= ax->deadband_deg) {
        PID_Reset(&s_pid[i]);
        ax->divergence_count = 0U;
        ax->no_progress_count = 0U;
        return 0;
    }

    bool     large    = (abs_err > CL_LARGE_ERR_DEG);
    uint32_t cooldown = large ? CL_COOLDOWN_HI_MS : CL_COOLDOWN_LO_MS;
    if (now_ms - s_last_correct_ms[i] < cooldown) return 0;

    float corr_deg;
    if (abs_err >= CL_RECOVERY_TRIGGER_DEG) {
        /* Recover most of a substantial missed-step error in one bounded,
           planned trajectory.  No new G-code command is needed. */
        corr_deg = error * CL_RECOVERY_TARGET_FRACTION;
        if (corr_deg >  CL_RECOVERY_MAX_DEG) corr_deg =  CL_RECOVERY_MAX_DEG;
        if (corr_deg < -CL_RECOVERY_MAX_DEG) corr_deg = -CL_RECOVERY_MAX_DEG;
        PID_Reset(&s_pid[i]);
    } else {
        float out_max = large ? CL_OUTPUT_MAX_HI : CL_OUTPUT_MAX_LO;
        if (out_max > CL_MAX_CORRECTION_DEG)
            out_max = CL_MAX_CORRECTION_DEG;
        corr_deg = PID_Compute(ax, &s_pid[i], error,
                               (float)CL_UPDATE_MS * 0.001f, out_max);
    }
    if (fabsf(corr_deg) < 0.01f) return 0;

    ax->pending_error_abs = abs_err;
    ax->correction_pending = true;
    ax->pending_is_recovery = (abs_err >= CL_RECOVERY_TRIGGER_DEG);
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
        s_axis[i].pending_error_abs = 0.0f;
        s_axis[i].divergence_count = 0U;
        s_axis[i].no_progress_count = 0U;
        s_axis[i].correction_pending = false;
        s_axis[i].pending_is_recovery = false;

        s_pid[i] = (PID_State_t){.integral_max = CL_INTEGRAL_MAX};
        s_last_correct_ms[i] = 0;
    }
    s_was_busy = false;
    s_settle_until_ms = 0U;
    s_recovery = (RecoveryTrajectory_t){0};
}

void Ctrl_ClosedLoop_SyncTarget(void)
{
    int32_t t[CL_AXIS_COUNT];
    Ctrl_MotionEngine_GetTheorySteps(&t[0], &t[1], &t[2]);
    uint32_t now = PlatformTime_NowMs();

    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        s_axis[i].target_deg = StepsToDeg(t[i]);
        s_axis[i].enabled    = true;
        s_axis[i].filt_valid = false;
        s_axis[i].correction_pending = false;
        s_axis[i].divergence_count = 0U;
        s_axis[i].no_progress_count = 0U;
        s_axis[i].pending_is_recovery = false;
        PID_Reset(&s_pid[i]);
        s_last_correct_ms[i] = now;

        UpdateEncoder(i);
    }
    s_was_busy = false;
    s_settle_until_ms = now + CL_SETTLE_MS;
    s_recovery = (RecoveryTrajectory_t){0};
}

void Ctrl_ClosedLoop_Update(void)
{
    if (Ctrl_MotionEngine_HasFault()) return;
    uint32_t now = PlatformTime_NowMs();

    /* Keep a long correction trajectory fed while it is executing.  Unlike
       a one-shot ring-buffer fill, this has no artificial break between
       chunks, regardless of the magnitude of the encoder error. */
    if (s_recovery.active && !RecoveryTrajectory_Fill()) return;

    bool     busy = Ctrl_MotionEngine_IsRunning() || Ctrl_MotionEngine_GetQueueCount() > 0;

    /* Always track multi-turn wraps, even when motion engine is busy.
       Otherwise a Nyquist gap opens during PID correction when the user
       is still forcing the motor. / 濮嬬粓杩借釜澶氬湀瓒婄�? 闃叉PID淇鏈熼棿
       鐢ㄦ埛鎸佺画鏆村姏鏃嬭浆瀵艰嚧鐨凬yquist閲囨牱缂哄彛�?*/
    if (busy) {
        s_was_busy = true;
        for (int i = 0; i < CL_AXIS_COUNT; i++) {
            if (s_axis[i].enabled)
                UpdateEncoder(i);
        }
        return;
    }

    /* The final recovery frame has completed.  Leave the existing target in
       place and use the normal settle/filter reset before evaluating error. */
    if (s_recovery.active) s_recovery.active = false;

    if (s_was_busy) {
        s_was_busy = false;
        s_settle_until_ms = now + CL_SETTLE_MS;
        for (int i = 0; i < CL_AXIS_COUNT; i++) {
            s_axis[i].filt_valid = false;
            PID_Reset(&s_pid[i]);
        }
        return;
    }
    if ((int32_t)(now - s_settle_until_ms) < 0) return;

    int32_t  s[CL_AXIS_COUNT];
    bool     any = false;

    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        s[i] = ComputeCorrection(i, now);
        if (s[i]) any = true;
    }

    if (!any) return;

    if (RecoveryTrajectory_Start(s)) {
        /* This motion returns the physical arm to its existing target; it is
           not a new commanded position.  Therefore it must not change the
           planner's/theory position, or the next G-code move would inherit a
           false offset after a successful missed-step recovery. */
        for (int i = 0; i < CL_AXIS_COUNT; i++)
            if (s[i]) s_last_correct_ms[i] = now;
    } else {
        for (int i = 0; i < CL_AXIS_COUNT; i++)
            if (s[i]) {
                s_axis[i].correction_pending = false;
                s_axis[i].pending_is_recovery = false;
            }
    }
}

/* 鈹€鈹€ Accessor functions / 璁块棶鍣ㄥ嚱�?鈹€鈹€ */

bool Ctrl_ClosedLoop_IsRecoveryActive(void)
{
    return s_recovery.active;
}

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
