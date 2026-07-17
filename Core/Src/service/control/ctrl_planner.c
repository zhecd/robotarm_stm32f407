/**
 * @file    ctrl_planner.c
 * @brief   Cartesian trajectory planner implementation. / 笛卡尔轨迹规划器实现。
 * @ingroup control
 */

#include "control/ctrl_planner.h"
#include "control/ctrl_kinematics.h"
#include "control/ctrl_motion_engine.h"
#include "common.h"
#include "main.h"
#include <stdlib.h>
#include <math.h>

/* ── Quintic smoothstep: zero velocity/accel at endpoints / 五次平滑: 端点速度/加速度为零 ── */
static float Smoothstep(float u)
{
    if (u <= 0.0f) return 0.0f;
    if (u >= 1.0f) return 1.0f;
    float u3 = u * u * u;
    float u4 = u3 * u;
    float u5 = u4 * u;
    return 6.0f * u5 - 15.0f * u4 + 10.0f * u3;
}

static int32_t s_planned_m1 = 0;
static int32_t s_planned_m2 = 0;
static int32_t s_planned_m3 = 0;
static float   s_cur_x      = 0.0f;
static float   s_cur_y      = 0.0f;
static float   s_cur_z      = 0.0f;

static void ApplyStepRateLimit(MotionFrame_t *frame)
{
    uint32_t steps = Common_MaxAbs3(frame->delta_m1, frame->delta_m2, frame->delta_m3);
    if (steps == 0U) return;

    uint64_t numerator = (uint64_t)steps * TICKS_PER_MS * 1000U;
    uint32_t min_ticks = (uint32_t)((numerator + MOTOR_MAX_STEP_RATE_HZ - 1U) /
                                    MOTOR_MAX_STEP_RATE_HZ);
    if (frame->total_ticks < min_ticks)
        frame->total_ticks = min_ticks;
}

ErrorCode_t Ctrl_Planner_Init(float start_x, float start_y, float start_z)
{
    s_cur_x = start_x;
    s_cur_y = start_y;
    s_cur_z = start_z;

    RobotAngles_t    ang;
    RobotMotorUnits_t units;
    ErrorCode_t status = Ctrl_Kinematics_Solve(start_x, start_y, start_z, &ang);
    if (status != ERR_OK) return status;
    Ctrl_Kinematics_ToMotorUnits(&ang, &units);
    s_planned_m1 = units.rot_units;
    s_planned_m2 = units.low_units;
    s_planned_m3 = units.high_units;
    return ERR_OK;
}

static ErrorCode_t PushFrameWithTimeout(const MotionFrame_t *frame)
{
    uint32_t start = HAL_GetTick();
    while (!Ctrl_MotionEngine_PushFrame(frame)) {
        if (Ctrl_MotionEngine_HasFault()) return ERR_BUSY;
        if ((HAL_GetTick() - start) >= PLANNER_QUEUE_TIMEOUT_MS) {
            Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_QUEUE_TIMEOUT);
            return ERR_TIMEOUT;
        }
    }
    return ERR_OK;
}

ErrorCode_t Ctrl_Planner_MoveLine(float target_x, float target_y, float target_z,
                                  uint32_t duration_ms)
{
    if (Ctrl_MotionEngine_HasFault()) return ERR_BUSY;
    const float sx = s_cur_x;
    const float sy = s_cur_y;
    const float sz = s_cur_z;
    const float dx = target_x - sx;
    const float dy = target_y - sy;
    const float dz = target_z - sz;
    const float dist = sqrtf(dx * dx + dy * dy + dz * dz);

    if (dist < 0.1f) {
        s_cur_x = target_x; s_cur_y = target_y; s_cur_z = target_z;
        return ERR_OK;
    }

    uint32_t total_ticks = duration_ms * TICKS_PER_MS;
    if (total_ticks < MIN_FRAME_TICKS)
        total_ticks = MIN_FRAME_TICKS;

    /* Segment count: distance-based, motor-step-based, and time-based / 分段数: 基于距离、步数和时间 */
    uint32_t seg_dist = (uint32_t)ceilf(dist / LINEAR_SEGMENT_MM);
    if (seg_dist == 0U) seg_dist = 1U;

    RobotAngles_t    fang;
    RobotMotorUnits_t funits;
    ErrorCode_t status = Ctrl_Kinematics_Solve(target_x, target_y, target_z, &fang);
    if (status != ERR_OK) return status;
    Ctrl_Kinematics_ToMotorUnits(&fang, &funits);

    MotionFrame_t total_move = {
        .delta_m1    = funits.rot_units  - s_planned_m1,
        .delta_m2    = funits.low_units  - s_planned_m2,
        .delta_m3    = funits.high_units - s_planned_m3,
        .total_ticks = 0U
    };
    uint32_t max_delta = Common_MaxAbs3(total_move.delta_m1, total_move.delta_m2, total_move.delta_m3);
    uint32_t seg_motor = (max_delta + MOTOR_STEPS_PER_SEG - 1U) / MOTOR_STEPS_PER_SEG;
    if (seg_motor == 0U) seg_motor = 1U;

    uint32_t tick_per_seg = PLANNER_SEGMENT_MS * TICKS_PER_MS;
    uint32_t seg_time = (total_ticks + tick_per_seg - 1U) / tick_per_seg;
    if (seg_time == 0U) seg_time = 1U;

    uint32_t segments = seg_dist;
    if (seg_time  > segments) segments = seg_time;
    if (seg_motor > segments) segments = seg_motor;
    if (segments  == 0U)     segments = 1U;

    /* Validate the whole Cartesian path before queuing any frame.  This
       prevents a partially executed move if an intermediate point crosses an
       unreachable region. */
    for (uint32_t i = 1; i <= segments; i++) {
        float progress = Smoothstep((float)i / (float)segments);
        RobotAngles_t check_angles;
        status = Ctrl_Kinematics_Solve(sx + dx * progress,
                                       sy + dy * progress,
                                       sz + dz * progress,
                                       &check_angles);
        if (status != ERR_OK) return status;
    }

    uint32_t prev_tick = 0U;

    for (uint32_t i = 1; i <= segments; i++) {
        float progress = Smoothstep((float)i / (float)segments);
        float step_x   = sx + dx * progress;
        float step_y   = sy + dy * progress;
        float step_z   = sz + dz * progress;

        uint32_t cum_ticks  = (uint32_t)(((uint64_t)total_ticks * i) / segments);
        uint32_t frame_ticks = cum_ticks - prev_tick;
        if (frame_ticks < MIN_FRAME_TICKS)
            frame_ticks = MIN_FRAME_TICKS;
        prev_tick = cum_ticks;

        RobotAngles_t    ta;
        RobotMotorUnits_t tu;
        status = Ctrl_Kinematics_Solve(step_x, step_y, step_z, &ta);
        if (status != ERR_OK) return status;
        Ctrl_Kinematics_ToMotorUnits(&ta, &tu);

        MotionFrame_t frame = {
            .delta_m1    = tu.rot_units  - s_planned_m1,
            .delta_m2    = tu.low_units  - s_planned_m2,
            .delta_m3    = tu.high_units - s_planned_m3,
            .total_ticks = frame_ticks
        };

        if (frame.delta_m1 == 0 && frame.delta_m2 == 0 && frame.delta_m3 == 0)
            continue;

        uint32_t fd = Common_MaxAbs3(frame.delta_m1, frame.delta_m2, frame.delta_m3);
        if (fd + FRAME_STEP_MARGIN > frame.total_ticks)
            frame.total_ticks = fd + FRAME_STEP_MARGIN;
        ApplyStepRateLimit(&frame);

        uint32_t remain = segments - i;
        if (remain < END_SLOW_SEGMENTS)
            frame.total_ticks += (END_SLOW_SEGMENTS - remain) * STOP_TAIL_EXTRA_TICKS;

        status = PushFrameWithTimeout(&frame);
        if (status != ERR_OK) return status;

        Ctrl_MotionEngine_AdjustTheorySteps(frame.delta_m1, frame.delta_m2, frame.delta_m3);

        s_planned_m1 = tu.rot_units;
        s_planned_m2 = tu.low_units;
        s_planned_m3 = tu.high_units;
    }

    s_cur_x = target_x; s_cur_y = target_y; s_cur_z = target_z;
    return ERR_OK;
}

ErrorCode_t Ctrl_Planner_TeleopStep(float dx, float dy, float dz)
{
    if (Ctrl_MotionEngine_HasFault()) return ERR_BUSY;
    if (Ctrl_MotionEngine_GetQueueCount() >= 1U)
        return ERR_BUSY;

    float tx = s_cur_x + dx;
    float ty = s_cur_y + dy;
    float tz = s_cur_z + dz;

    RobotAngles_t    ta;
    RobotMotorUnits_t tu;
    ErrorCode_t status = Ctrl_Kinematics_Solve(tx, ty, tz, &ta);
    if (status != ERR_OK) return status;
    Ctrl_Kinematics_ToMotorUnits(&ta, &tu);

    MotionFrame_t frame = {
        .delta_m1    = tu.rot_units  - s_planned_m1,
        .delta_m2    = tu.low_units  - s_planned_m2,
        .delta_m3    = tu.high_units - s_planned_m3,
        .total_ticks = TELEOP_FRAME_MS * TICKS_PER_MS
    };

    if (frame.delta_m1 == 0 && frame.delta_m2 == 0 && frame.delta_m3 == 0)
        return ERR_OK;

    uint32_t fd = Common_MaxAbs3(frame.delta_m1, frame.delta_m2, frame.delta_m3);
    if (fd > frame.total_ticks)
        frame.total_ticks = fd;
    ApplyStepRateLimit(&frame);

    if (!Ctrl_MotionEngine_PushFrame(&frame))
        return Ctrl_MotionEngine_HasFault() ? ERR_BUSY : ERR_BUFFER_FULL;

    Ctrl_MotionEngine_AdjustTheorySteps(frame.delta_m1, frame.delta_m2, frame.delta_m3);

    s_planned_m1 = tu.rot_units;
    s_planned_m2 = tu.low_units;
    s_planned_m3 = tu.high_units;
    s_cur_x = tx; s_cur_y = ty; s_cur_z = tz;
    return ERR_OK;
}
