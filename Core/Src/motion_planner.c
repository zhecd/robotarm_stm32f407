#include "motion_planner.h"
#include "robotGeometry.h"
#include "motor_core.h"
#include "bsp_stepper.h"
#include <stdlib.h>
#include <math.h>

#define TICKS_PER_MS    50U
#define LINEAR_STEP_MM  1.0f
#define TELEOP_FRAME_MS 5U

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
    float dx = target_x - current_x;
    float dy = target_y - current_y;
    float dz = target_z - current_z;
    float distance = sqrtf(dx * dx + dy * dy + dz * dz);

    if (distance < 0.1f) return true;

    uint32_t segments = (uint32_t)(distance / LINEAR_STEP_MM);
    if (segments == 0U) segments = 1U;

    uint32_t base_ticks = (duration_ms * TICKS_PER_MS) / segments;
    if (base_ticks == 0U) base_ticks = 1U;

    const float ACCEL_ZONE = 0.2f;
    const float DECEL_ZONE = 0.2f;
    const float V_MIN_RATIO = 0.15f;

    for (uint32_t i = 1; i <= segments; i++) {
        float progress = (float)i / (float)segments;
        float step_x = current_x + dx * progress;
        float step_y = current_y + dy * progress;
        float step_z = current_z + dz * progress;

        float v_mult = 1.0f;
        if (progress < ACCEL_ZONE) {
            float u = progress / ACCEL_ZONE;
            v_mult = V_MIN_RATIO + (1.0f - V_MIN_RATIO) * Quintic_Smoothstep(u);
        } else if (progress > (1.0f - DECEL_ZONE)) {
            float u = (progress - (1.0f - DECEL_ZONE)) / DECEL_ZONE;
            v_mult = 1.0f - (1.0f - V_MIN_RATIO) * Quintic_Smoothstep(u);
        }

        uint32_t current_ticks = (uint32_t)(base_ticks / v_mult);

        RobotAngles target_angles;
        RobotMotorUnits target_units;
        MotionFrame_t frame;

        RobotGeometry_CalculateAngles(step_x, step_y, step_z, &target_angles);
        RobotGeometry_AnglesToMotorUnits(&target_angles, &target_units);

        frame.delta_m1 = target_units.rotUnits - planned_pos_m1;
        frame.delta_m2 = target_units.lowUnits - planned_pos_m2;
        frame.delta_m3 = target_units.highUnits - planned_pos_m3;
        frame.total_ticks = current_ticks;

        uint32_t max_delta = abs(frame.delta_m1);
        if ((uint32_t)abs(frame.delta_m2) > max_delta) max_delta = abs(frame.delta_m2);
        if ((uint32_t)abs(frame.delta_m3) > max_delta) max_delta = abs(frame.delta_m3);
        if (max_delta > frame.total_ticks) {
            frame.total_ticks = max_delta + 5U;
        }

        while (!Motor_Buffer_Push(&frame)) {}

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

    uint32_t max_delta = abs(frame.delta_m1);
    if ((uint32_t)abs(frame.delta_m2) > max_delta) max_delta = abs(frame.delta_m2);
    if ((uint32_t)abs(frame.delta_m3) > max_delta) max_delta = abs(frame.delta_m3);
    if (max_delta > frame.total_ticks) {
        frame.total_ticks = max_delta;
    }

    if (!Motor_Buffer_Push(&frame)) {
        return false;
    }

    planned_pos_m1 = target_units.rotUnits;
    planned_pos_m2 = target_units.lowUnits;
    planned_pos_m3 = target_units.highUnits;
    current_x = target_x;
    current_y = target_y;
    current_z = target_z;
    return true;
}
