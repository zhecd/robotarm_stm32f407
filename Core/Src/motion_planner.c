#include "motion_planner.h"
#include "robotGeometry.h"
#include "motor_core.h"
#include "bsp_stepper.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TICKS_PER_MS           50U
#define LINEAR_SEGMENT_MM      0.25f
#define PLANNER_SEGMENT_MS     4U
#define MIN_FRAME_TICKS        1U
#define FRAME_STEP_MARGIN      3U
#define STOP_TAIL_EXTRA_TICKS  2U
#define END_SLOW_SEGMENTS      2U
#define MOTOR_STEPS_PER_SEG    10U
#define TELEOP_FRAME_MS        5U

static float Quintic_Smoothstep(float u)
{
    if (u <= 0.0f) return 0.0f;
    if (u >= 1.0f) return 1.0f;
    float u3 = u * u * u;
    float u4 = u3 * u;
    float u5 = u4 * u;
    return 6.0f * u5 - 15.0f * u4 + 10.0f * u3;
}

static int32_t planned_pos_m1 = 0;
static int32_t planned_pos_m2 = 0;
static int32_t planned_pos_m3 = 0;
static float current_x = 0.0f;
static float current_y = 0.0f;
static float current_z = 0.0f;

static uint32_t MotionPlanner_MaxAbsDelta(const MotionFrame_t *frame)
{
    uint32_t max_delta = abs(frame->delta_m1);
    if ((uint32_t)abs(frame->delta_m2) > max_delta) max_delta = abs(frame->delta_m2);
    if ((uint32_t)abs(frame->delta_m3) > max_delta) max_delta = abs(frame->delta_m3);
    return max_delta;
}

void Motion_Planner_Init(float start_x, float start_y, float start_z)
{
    current_x = start_x;
    current_y = start_y;
    current_z = start_z;
    planned_pos_m1 = Motor_M1.absolute_position;
    planned_pos_m2 = Motor_M2.absolute_position;
    planned_pos_m3 = Motor_M3.absolute_position;
}

bool Motion_Planner_MoveLine(float target_x, float target_y, float target_z, uint32_t duration_ms)
{
    const float start_x = current_x;
    const float start_y = current_y;
    const float start_z = current_z;

    const float dx = target_x - start_x;
    const float dy = target_y - start_y;
    const float dz = target_z - start_z;
    const float distance = sqrtf(dx * dx + dy * dy + dz * dz);

    if (distance < 0.1f) {
        current_x = target_x;
        current_y = target_y;
        current_z = target_z;
        return true;
    }

    uint32_t total_ticks = duration_ms * TICKS_PER_MS;
    if (total_ticks < MIN_FRAME_TICKS) {
        total_ticks = MIN_FRAME_TICKS;
    }

    uint32_t segments_by_distance = (uint32_t)ceilf(distance / LINEAR_SEGMENT_MM);
    if (segments_by_distance == 0U) {
        segments_by_distance = 1U;
    }

    RobotAngles final_angles;
    RobotMotorUnits final_units;
    RobotGeometry_CalculateAngles(target_x, target_y, target_z, &final_angles);
    RobotGeometry_AnglesToMotorUnits(&final_angles, &final_units);

    MotionFrame_t total_move = {
        .delta_m1 = final_units.rotUnits - planned_pos_m1,
        .delta_m2 = final_units.lowUnits - planned_pos_m2,
        .delta_m3 = final_units.highUnits - planned_pos_m3,
        .total_ticks = 0U
    };

    uint32_t max_total_delta = MotionPlanner_MaxAbsDelta(&total_move);
    uint32_t segments_by_motor = (max_total_delta + MOTOR_STEPS_PER_SEG - 1U) / MOTOR_STEPS_PER_SEG;
    if (segments_by_motor == 0U) {
        segments_by_motor = 1U;
    }

    const uint32_t nominal_segment_ticks = PLANNER_SEGMENT_MS * TICKS_PER_MS;
    uint32_t segments_by_time = (total_ticks + nominal_segment_ticks - 1U) / nominal_segment_ticks;
    if (segments_by_time == 0U) {
        segments_by_time = 1U;
    }

    uint32_t segments = (segments_by_distance > segments_by_time) ? segments_by_distance : segments_by_time;
    if (segments_by_motor > segments) {
        segments = segments_by_motor;
    }
    if (segments == 0U) {
        segments = 1U;
    }

    uint32_t prev_tick_target = 0U;

    for (uint32_t i = 1; i <= segments; i++) {
        const float progress = (float)i / (float)segments;
        const float smooth_progress = Quintic_Smoothstep(progress);

        const float step_x = start_x + dx * smooth_progress;
        const float step_y = start_y + dy * smooth_progress;
        const float step_z = start_z + dz * smooth_progress;

        const uint32_t cumulative_ticks = (uint32_t)(((uint64_t)total_ticks * i) / segments);
        uint32_t frame_ticks = cumulative_ticks - prev_tick_target;
        if (frame_ticks < MIN_FRAME_TICKS) {
            frame_ticks = MIN_FRAME_TICKS;
        }
        prev_tick_target = cumulative_ticks;

        RobotAngles target_angles;
        RobotMotorUnits target_units;
        MotionFrame_t frame;

        RobotGeometry_CalculateAngles(step_x, step_y, step_z, &target_angles);
        RobotGeometry_AnglesToMotorUnits(&target_angles, &target_units);

        frame.delta_m1 = target_units.rotUnits - planned_pos_m1;
        frame.delta_m2 = target_units.lowUnits - planned_pos_m2;
        frame.delta_m3 = target_units.highUnits - planned_pos_m3;
        frame.total_ticks = frame_ticks;

        if ((frame.delta_m1 == 0) && (frame.delta_m2 == 0) && (frame.delta_m3 == 0)) {
            continue;
        }

        uint32_t max_delta = MotionPlanner_MaxAbsDelta(&frame);
        if (max_delta + FRAME_STEP_MARGIN > frame.total_ticks) {
            frame.total_ticks = max_delta + FRAME_STEP_MARGIN;
        }
        uint32_t remaining_segments = segments - i;
        if (remaining_segments < END_SLOW_SEGMENTS) {
            frame.total_ticks += (END_SLOW_SEGMENTS - remaining_segments) * STOP_TAIL_EXTRA_TICKS;
        }

        while (!Motor_Buffer_Push(&frame)) {}

        Motor_Core_AdjustTheorySteps(frame.delta_m1, frame.delta_m2, frame.delta_m3);

        planned_pos_m1 = target_units.rotUnits;
        planned_pos_m2 = target_units.lowUnits;
        planned_pos_m3 = target_units.highUnits;
    }

    current_x = target_x;
    current_y = target_y;
    current_z = target_z;
    return true;
}

bool Motion_Planner_TeleopStep(float dx, float dy, float dz)
{
    if (Motor_Buffer_GetCount() >= 1U) {
        return false;
    }

    float target_x = current_x + dx;
    float target_y = current_y + dy;
    float target_z = current_z + dz;

    RobotAngles target_angles;
    RobotMotorUnits target_units;
    MotionFrame_t frame;

    RobotGeometry_CalculateAngles(target_x, target_y, target_z, &target_angles);
    RobotGeometry_AnglesToMotorUnits(&target_angles, &target_units);

    frame.delta_m1 = target_units.rotUnits - planned_pos_m1;
    frame.delta_m2 = target_units.lowUnits - planned_pos_m2;
    frame.delta_m3 = target_units.highUnits - planned_pos_m3;

    if ((frame.delta_m1 == 0) && (frame.delta_m2 == 0) && (frame.delta_m3 == 0)) {
        return false;
    }

    frame.total_ticks = TELEOP_FRAME_MS * TICKS_PER_MS;

    uint32_t max_delta = MotionPlanner_MaxAbsDelta(&frame);
    if (max_delta > frame.total_ticks) {
        frame.total_ticks = max_delta;
    }

    if (!Motor_Buffer_Push(&frame)) {
        return false;
    }

    Motor_Core_AdjustTheorySteps(frame.delta_m1, frame.delta_m2, frame.delta_m3);

    planned_pos_m1 = target_units.rotUnits;
    planned_pos_m2 = target_units.lowUnits;
    planned_pos_m3 = target_units.highUnits;
    current_x = target_x;
    current_y = target_y;
    current_z = target_z;
    return true;
}
