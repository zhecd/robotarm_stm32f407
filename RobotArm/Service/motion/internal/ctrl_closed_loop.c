/**
 * @file ctrl_closed_loop.c
 * @brief 闭环内部实现：编码器采样、误差补偿和补偿发散保护。
 * @details 补偿使用电机轴角度，恢复原有目标，不修改规划器的理论位置。
 */

#include "ctrl_closed_loop.h"
#include "ctrl_motion_engine.h"
#include "state_service.h"
#include "device/dev_joint.h"
#include "platform_time.h"
#include "robot_home_pose.h"
#include "robot_math.h"
#include "error_code.h"
#include <math.h>
#include <stdio.h>

/* 每轴 PID 私有状态。 */
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
    float          pending_command_abs;
    bool           recovering;
    uint32_t       recovery_started_ms;
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
static bool          s_settle_pending;
static RecoveryTrajectory_t s_recovery;
/* 诊断只记录已经发生的判定/受理，不参与控制决策；Init开始新统计会话。 */
static ClosedLoopStatus_t s_status;

/* 恢复会话跨越多轮轨迹和稳定等待；到位/新目标/停机时撤销，防止旧计时串入新任务。 */
static void ResetRecovery(AxisCL_t *ax)
{
    ax->recovering = false;
    ax->correction_pending = false;
    ax->pending_error_abs = 0.0f;
    ax->pending_command_abs = 0.0f;
    ax->divergence_count = 0U;
    ax->no_progress_count = 0U;
    ax->recovery_started_ms = 0U;
}

static bool CheckRecoveryWindow(const AxisCL_t *ax, float motor_angle)
{
    /* 同时检查未滤波反馈，避免EMA把突然超窗的误差暂时掩盖。
     * 只在补偿会话/闲置计算中使用，正常G1执行中不拿尚未到达的目标判丢步。 */
    if (fabsf(ax->target_deg - motor_angle) > CL_RECOVERY_WINDOW_JOINT_DEG * GEAR_RATIO) {
        Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_CORRECTION_RANGE);
        return false;
    }
    return true;
}

static bool UpdateEncoder(int axis)
{
    AxisCL_t *ax = &s_axis[axis];
    float motor_angle_deg = 0.0f;
    uint32_t started_ms=PlatformTime_NowMs();
    if (Dev_Joint_ReadMotorAngle((DevJointId_t)axis, &motor_angle_deg) == ERR_OK &&
        isfinite(motor_angle_deg) &&
        (uint32_t)(PlatformTime_NowMs()-started_ms)<CL_FEEDBACK_MAX_READ_MS) {
#if JOINT_LIMITS_ENABLED
        if (!Dev_Joint_IsWithinSoftLimit((DevJointId_t)axis, motor_angle_deg)) {
            StateService_PublishAxisSampleTimed((uint8_t)axis, motor_angle_deg,
                RobotHomePose_MotorDegToJointDeg((RobotHomeAxis_t)axis, motor_angle_deg),started_ms,PlatformTime_NowMs());
            Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_SOFT_LIMIT);
            return false;
        }
#endif
        ax->motor_angle_deg = motor_angle_deg;
        ax->read_failures = 0U;
        StateService_PublishAxisSampleTimed((uint8_t)axis, motor_angle_deg,
            RobotHomePose_MotorDegToJointDeg((RobotHomeAxis_t)axis, motor_angle_deg),started_ms,PlatformTime_NowMs());
        if (ax->recovering && !CheckRecoveryWindow(ax, motor_angle_deg)) return false;
        return true;
    }
    if(ax->read_failures<UINT8_MAX)ax->read_failures++;
    StateService_PublishAxisReadFailure((uint8_t)axis, ax->read_failures);
    if (ax->read_failures >= CL_ENCODER_FAIL_LIMIT)
        Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_ENCODER);
    s_status.decision_mask &= (uint8_t)~(1U << axis);
    return false;
}

/* PID 计算。 */

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

    /* 五次平滑曲线：两端速度、加速度为零，补偿也经过逐段缓起缓停。
       这里计算归一化位置比例，不直接操作STEP引脚。 */
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

        /* 零步数帧仍占用时间，保留曲线两端低速区；丢掉它们会改变速度曲线。 */
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

/* 逐轴计算补偿量。 */

static int32_t ComputeCorrection(int i, uint32_t now_ms)
{
    AxisCL_t *ax = &s_axis[i];
    if (!ax->enabled) return 0;

    if (!UpdateEncoder(i)) return 0;
    if (!CheckRecoveryWindow(ax, ax->motor_angle_deg)) return 0;

    /* 目标与反馈统一用电机轴角度deg。规划器按关节传动比生成微步，
       StepsToDeg再按电机微步角换回电机轴角；不能把关节角直接代入误差。 */
    float raw = ax->motor_angle_deg;
    if (!ax->filt_valid) {
        ax->filt_angle = raw;
        ax->filt_valid = true;
    } else {
        ax->filt_angle += CL_EMA_ALPHA * (raw - ax->filt_angle);
    }

    float error   = ax->target_deg - ax->filt_angle;
    float abs_err = fabsf(error);

    /* 与本次控制计算使用同一份EMA误差，不在查询时重新读编码器或推进PID。
     * 时间取读取前的now_ms，使耗时读取不会被误标为刚刚完成的新样本。 */
    s_status.error_motor_deg[i] = error;
    s_status.decision_ms[i] = now_ms;
    s_status.decision_mask |= (uint8_t)(1U << i);

    /* 成功优先：本轮已经到位时不再以改善量门槛误判失败，死区并未扩大到10°。 */
    if (abs_err <= ax->deadband_deg) {
        PID_Reset(&s_pid[i]);
        ResetRecovery(ax);
        return 0;
    }
    if (!ax->recovering) {
        ax->recovering = true;
        ax->recovery_started_ms = now_ms;
    }

    if (ax->correction_pending) {
        if (abs_err > ax->pending_error_abs + CL_DIVERGENCE_TOLERANCE_DEG) {
            if (++ax->divergence_count >= CL_DIVERGENCE_LIMIT) {
                Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_CONTROL_DIVERGENCE);
                return 0;
            }
        } else {
            ax->divergence_count = 0U;
            float required = ax->pending_command_abs * CL_RECOVERY_PROGRESS_FRACTION;
            if (required > CL_RECOVERY_MIN_PROGRESS_DEG) required = CL_RECOVERY_MIN_PROGRESS_DEG;
            if ((ax->pending_error_abs - abs_err) < required) {
                if (++ax->no_progress_count >= CL_RECOVERY_NO_PROGRESS_LIMIT) {
                    Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_CORRECTION_STALLED);
                    return 0;
                }
            } else {
                ax->no_progress_count = 0U;
            }
        }
        ax->correction_pending = false;
    }

    bool     large    = (abs_err > CL_LARGE_ERR_DEG);
    uint32_t cooldown = large ? CL_COOLDOWN_HI_MS : CL_COOLDOWN_LO_MS;
    if (now_ms - s_last_correct_ms[i] < cooldown) return 0;

    float corr_deg;
    if (abs_err >= CL_RECOVERY_TRIGGER_DEG) {
        /* 大误差走有界恢复轨迹，无需新G-code；每轮最多1°关节角。
           真实恢复能力仍取决于负载、限流与机构，不能用软件轨迹代替实测。 */
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
    float round_max = CL_RECOVERY_ROUND_JOINT_DEG * GEAR_RATIO;
    if (corr_deg > round_max) corr_deg = round_max;
    if (corr_deg < -round_max) corr_deg = -round_max;
    if (fabsf(corr_deg) < 0.01f) return 0;

    int32_t steps = DegToSteps(corr_deg);
    if (steps == 0) return 0;

    ax->pending_error_abs = abs_err;
    ax->correction_pending = true;
    ax->pending_command_abs = fabsf(StepsToDeg(steps));
    return steps;
}

/* 公开接口。 */

void Ctrl_ClosedLoop_Init(void)
{
    /* 正常回零先重置Motion理论微步并标定编码器，再以Init建立默认零目标，
     * 首条G1之前不会调用SyncTarget。诊断必须承认这个已有目标，不能误报无参考。
     * 这里只初始化记录，不增加采样、PID或STEP；真正参考仍由App结合Safety判定。 */
    s_status = (ClosedLoopStatus_t){.target_initialized = true};
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
        ResetRecovery(&s_axis[i]);

        s_pid[i] = (PID_State_t){.integral_max = CL_INTEGRAL_MAX};
        s_last_correct_ms[i] = 0;
    }
    s_was_busy = false;
    s_settle_until_ms = 0U;
    s_settle_pending = false;
    s_recovery = (RecoveryTrajectory_t){0};
}

void Ctrl_ClosedLoop_SyncTarget(void)
{
    s_status.target_initialized = true;
    s_status.decision_mask = 0U;
    int32_t t[CL_AXIS_COUNT];
    Ctrl_MotionEngine_GetTheorySteps(&t[0], &t[1], &t[2]);
    uint32_t now = PlatformTime_NowMs();

    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        s_axis[i].target_deg = StepsToDeg(t[i]);
        s_axis[i].enabled    = true;
        s_axis[i].filt_valid = false;
        ResetRecovery(&s_axis[i]);
        PID_Reset(&s_pid[i]);
        s_last_correct_ms[i] = now;

        UpdateEncoder(i);
    }
    s_was_busy = false;
    s_settle_until_ms = now + CL_SETTLE_MS;
    s_settle_pending = true;
    s_recovery = (RecoveryTrajectory_t){0};
}

void Ctrl_ClosedLoop_Update(void)
{
    if (Ctrl_MotionEngine_HasFault()) {
        /* STEP引擎已清队列；同步撤销尚未生成的补偿段，状态查询不能一直显示恢复中。
         * 不清故障、不修改目标，也不恢复许可；重新回零后由Init/SyncTarget建立新参考。 */
        s_recovery = (RecoveryTrajectory_t){0};
        for (int i = 0; i < CL_AXIS_COUNT; i++) {
            ResetRecovery(&s_axis[i]);
        }
        return;
    }
    uint32_t now = PlatformTime_NowMs();
    /* 总截止包含正在消费的帧和250ms稳定等待，不能靠不断分段刷新期限。
     * unsigned差值支持毫秒计数器回绕；故障由引擎立即清队列、禁止STEP。 */
    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        if (s_axis[i].enabled && s_axis[i].recovering &&
            (uint32_t)(now - s_axis[i].recovery_started_ms) >= CL_RECOVERY_TIMEOUT_MS) {
            Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_CORRECTION_TIMEOUT);
            return;
        }
    }

    /* 主循环分批补充恢复帧，保持有限预填深度；STEP中断只消费已准备帧。
       长补偿不要求一次装入整个轨迹，也不因一次队列容量限制而截断。 */
    if (s_recovery.active && !RecoveryTrajectory_Fill()) return;

    bool     busy = Ctrl_MotionEngine_IsRunning() || Ctrl_MotionEngine_GetQueueCount() > 0;

    /* 运动期间仍持续采样，避免软件多圈跟踪因采样间隔过长而丢失圈数。 */
    if (busy) {
        s_was_busy = true;
        for (int i = 0; i < CL_AXIS_COUNT; i++) {
            if (s_axis[i].enabled) UpdateEncoder(i);
            if (Ctrl_MotionEngine_HasFault()) return;
        }
        return;
    }

    /* 最后一帧已完成。保留原目标，等待机械稳定并重置滤波后再判断误差。 */
    if (s_recovery.active) s_recovery.active = false;

    if (s_was_busy) {
        s_was_busy = false;
        s_settle_until_ms = now + CL_SETTLE_MS;
        s_settle_pending = true;
        s_status.decision_mask = 0U;
        for (int i = 0; i < CL_AXIS_COUNT; i++) {
            s_axis[i].filt_valid = false;
            PID_Reset(&s_pid[i]);
        }
    }
    if (s_settle_pending && (int32_t)(now - s_settle_until_ms) < 0) {
        /* 250ms稳定等待仅禁止生成补偿，采样仍按主循环闭环周期执行。
           若此处直接返回，M400后的反馈会超过100ms有效期，上位机便会
           正确拒绝下一段运动。继续发布实测值，不改目标、不推进PID，
           同时保留读取失败与软限位保护；不能以放宽新鲜度代替采样。 */
        for (int i = 0; i < CL_AXIS_COUNT; i++) {
            if (s_axis[i].enabled)
                UpdateEncoder(i);
            if (Ctrl_MotionEngine_HasFault()) return;
        }
        return;
    }

    s_settle_pending = false;
    int32_t  s[CL_AXIS_COUNT];
    bool     any = false;

    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        s[i] = ComputeCorrection(i, now);
        if (Ctrl_MotionEngine_HasFault()) return;
        if (s[i]) any = true;
    }

    if (!any) return;

    if (RecoveryTrajectory_Start(s)) {
        /* 补偿返回原目标，不是新的业务位移。因此不修改规划器理论位置，
           否则下一条G-code会继承补偿造成的假偏移。 */
        for (int i = 0; i < CL_AXIS_COUNT; i++)
            if (s[i]) {
                s_last_correct_ms[i] = now;
                /* 受理不代表执行完毕：急停可能撤销剩余帧，不能把此数当STEP计数。 */
                if (s_status.accepted_rounds[i] < UINT32_MAX)
                    s_status.accepted_rounds[i]++;
                s_status.last_steps[i] = s[i];
            }
    } else {
        for (int i = 0; i < CL_AXIS_COUNT; i++)
            if (s[i]) {
                s_axis[i].correction_pending = false;
            }
    }
}

/* 设备实例访问入口。 */

bool Ctrl_ClosedLoop_IsRecoveryActive(void)
{
    /* 停止可能发生在下一轮Update之前，查询也必须立即反映动作已撤销。 */
    return s_recovery.active && !Ctrl_MotionEngine_HasFault();
}

bool Ctrl_ClosedLoop_HasSettled(void)
{
    const uint8_t required = (uint8_t)((1U << CL_AXIS_COUNT) - 1U);
    if (!s_status.target_initialized || s_was_busy || s_settle_pending ||
        s_recovery.active || Ctrl_MotionEngine_HasFault() ||
        (s_status.decision_mask & required) != required) return false;

    StateServiceStatus_t feedback;
    StateService_GetStatus(&feedback);
    const uint32_t now = PlatformTime_NowMs();
    if ((StateService_FreshMask(&feedback, now, FEEDBACK_QUERY_MAX_AGE_MS) &
         required) != required) return false;

    for (int i = 0; i < CL_AXIS_COUNT; ++i) {
        const AxisCL_t *axis = &s_axis[i];
        if (!axis->enabled || !axis->filt_valid || axis->recovering || axis->correction_pending ||
            (uint32_t)(now - s_status.decision_ms[i]) >= FEEDBACK_QUERY_MAX_AGE_MS ||
            !(fabsf(s_status.error_motor_deg[i]) <= axis->deadband_deg) ||
            !(fabsf(axis->target_deg - axis->motor_angle_deg) <= axis->deadband_deg))
            return false;
    }
    return true;
}

void Ctrl_ClosedLoop_GetStatus(ClosedLoopStatus_t *out)
{
    if (!out) return;
    *out = s_status;
    out->target_initialized = s_status.target_initialized && !Ctrl_MotionEngine_HasFault();
    out->active = Ctrl_ClosedLoop_IsRecoveryActive();
    out->enabled_mask = 0U;
    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        out->target_motor_deg[i] = s_axis[i].target_deg;
        if (s_axis[i].enabled) out->enabled_mask |= (uint8_t)(1U << i);
        /* 轨迹完成后的稳定等待会撤销滤波。保留旧数值便于取证，但不标有效。 */
        if (!s_axis[i].filt_valid || !out->target_initialized)
            out->decision_mask &= (uint8_t)~(1U << i);
    }
}

bool Ctrl_ClosedLoop_IsAxisEnabled(int axis)
{
    return (axis >= 0 && axis < CL_AXIS_COUNT) ? s_axis[axis].enabled : false;
}

void Ctrl_ClosedLoop_SetAxisEnabled(int axis, bool en)
{
    if (axis >= 0 && axis < CL_AXIS_COUNT) {
        s_axis[axis].enabled = en;
        ResetRecovery(&s_axis[axis]);
        s_axis[axis].filt_valid = false;
    }
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
