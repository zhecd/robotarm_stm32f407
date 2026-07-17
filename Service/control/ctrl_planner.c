/**
 * @file ctrl_planner.c
 * @brief Incremental Cartesian path validation and motion-frame generation.
 */

#include "service/control/ctrl_planner.h"

#include "main.h"
#include "robot_math.h"
#include "service/control/ctrl_kinematics.h"
#include "service/control/ctrl_motion_engine.h"

#include <math.h>

#define PLANNER_SERVICE_FRAME_BUDGET       4U
#define PLANNER_VALIDATE_SEGMENT_BUDGET   16U
#define PLANNER_MAX_SEGMENTS             4096U

typedef enum {
    PLANNER_IDLE = 0,
    PLANNER_VALIDATING,
    PLANNER_STREAMING
} PlannerState_t;

typedef struct {
    PlannerState_t state;
    float start_x;
    float start_y;
    float start_z;
    float delta_x;
    float delta_y;
    float delta_z;
    float target_x;
    float target_y;
    float target_z;
    uint32_t total_ticks;
    uint32_t segments;
    uint32_t validate_index;
    uint32_t generate_index;
    uint32_t previous_tick;
    int32_t start_m1;
    int32_t start_m2;
    int32_t start_m3;
    int32_t target_m1;
    int32_t target_m2;
    int32_t target_m3;
    int32_t generated_m1;
    int32_t generated_m2;
    int32_t generated_m3;
} PlannerMove_t;

static int32_t s_planned_m1;
static int32_t s_planned_m2;
static int32_t s_planned_m3;
static float s_cur_x;
static float s_cur_y;
static float s_cur_z;
static PlannerMove_t s_move;
static bool s_start_result_ready;
static ErrorCode_t s_start_result;

static float Smoothstep(float u)
{
    if (u <= 0.0f) return 0.0f;
    if (u >= 1.0f) return 1.0f;
    float u3 = u * u * u;
    float u4 = u3 * u;
    float u5 = u4 * u;
    return 6.0f * u5 - 15.0f * u4 + 10.0f * u3;
}

static void ApplyStepRateLimit(MotionFrame_t *frame)
{
    uint32_t steps = RobotMath_MaxAbs3(frame->delta_m1, frame->delta_m2,
                                        frame->delta_m3);
    if (steps == 0U) return;

    uint64_t numerator = (uint64_t)steps * TICKS_PER_MS * 1000U;
    uint32_t min_ticks = (uint32_t)((numerator + MOTOR_MAX_STEP_RATE_HZ - 1U) /
                                    MOTOR_MAX_STEP_RATE_HZ);
    if (frame->total_ticks < min_ticks) frame->total_ticks = min_ticks;
}

static void PublishStartResult(ErrorCode_t result)
{
    s_start_result = result;
    s_start_result_ready = true;
}

ErrorCode_t Ctrl_Planner_Init(float start_x, float start_y, float start_z)
{
    RobotAngles_t angles;
    RobotMotorUnits_t units;
    ErrorCode_t status = Ctrl_Kinematics_Solve(start_x, start_y, start_z, &angles);
    if (status != ERR_OK) return status;

    Ctrl_Kinematics_ToMotorUnits(&angles, &units);
    s_cur_x = start_x;
    s_cur_y = start_y;
    s_cur_z = start_z;
    s_planned_m1 = units.rot_units;
    s_planned_m2 = units.low_units;
    s_planned_m3 = units.high_units;
    s_move.state = PLANNER_IDLE;
    s_start_result_ready = false;
    s_start_result = ERR_OK;
    return ERR_OK;
}

bool Ctrl_Planner_IsBusy(void)
{
    return s_move.state != PLANNER_IDLE;
}

bool Ctrl_Planner_TakeStartResult(ErrorCode_t *out_result)
{
    if (!s_start_result_ready) return false;
    if (out_result) *out_result = s_start_result;
    s_start_result_ready = false;
    return true;
}

ErrorCode_t Ctrl_Planner_MoveLine(float target_x, float target_y, float target_z,
                                  uint32_t duration_ms)
{
    if (Ctrl_MotionEngine_HasFault() || Ctrl_Planner_IsBusy()) return ERR_BUSY;

    const float dx = target_x - s_cur_x;
    const float dy = target_y - s_cur_y;
    const float dz = target_z - s_cur_z;
    const float distance = sqrtf(dx * dx + dy * dy + dz * dz);
    if (distance < 0.1f) {
        s_cur_x = target_x;
        s_cur_y = target_y;
        s_cur_z = target_z;
        return ERR_OK;
    }

    RobotAngles_t target_angles;
    RobotMotorUnits_t target_units;
    ErrorCode_t status = Ctrl_Kinematics_Solve(target_x, target_y, target_z,
                                                &target_angles);
    if (status != ERR_OK) return status;
    Ctrl_Kinematics_ToMotorUnits(&target_angles, &target_units);

    uint32_t total_ticks = duration_ms * TICKS_PER_MS;
    if (total_ticks < MIN_FRAME_TICKS) total_ticks = MIN_FRAME_TICKS;

    uint32_t seg_dist = (uint32_t)ceilf(distance / LINEAR_SEGMENT_MM);
    if (seg_dist == 0U) seg_dist = 1U;

    uint32_t max_delta = RobotMath_MaxAbs3(target_units.rot_units - s_planned_m1,
                                            target_units.low_units - s_planned_m2,
                                            target_units.high_units - s_planned_m3);
    uint32_t seg_motor = (max_delta + MOTOR_STEPS_PER_SEG - 1U) /
                         MOTOR_STEPS_PER_SEG;
    if (seg_motor == 0U) seg_motor = 1U;

    uint32_t ticks_per_segment = PLANNER_SEGMENT_MS * TICKS_PER_MS;
    uint32_t seg_time = (total_ticks + ticks_per_segment - 1U) /
                        ticks_per_segment;
    if (seg_time == 0U) seg_time = 1U;

    uint32_t segments = seg_dist;
    if (seg_time > segments) segments = seg_time;
    if (seg_motor > segments) segments = seg_motor;
    if (segments > PLANNER_MAX_SEGMENTS) return ERR_OUT_OF_RANGE;

    s_move = (PlannerMove_t){
        .state = PLANNER_VALIDATING,
        .start_x = s_cur_x,
        .start_y = s_cur_y,
        .start_z = s_cur_z,
        .delta_x = dx,
        .delta_y = dy,
        .delta_z = dz,
        .target_x = target_x,
        .target_y = target_y,
        .target_z = target_z,
        .total_ticks = total_ticks,
        .segments = segments,
        .validate_index = 1U,
        .generate_index = 1U,
        .start_m1 = s_planned_m1,
        .start_m2 = s_planned_m2,
        .start_m3 = s_planned_m3,
        .target_m1 = target_units.rot_units,
        .target_m2 = target_units.low_units,
        .target_m3 = target_units.high_units,
        .generated_m1 = s_planned_m1,
        .generated_m2 = s_planned_m2,
        .generated_m3 = s_planned_m3,
    };
    s_start_result_ready = false;
    return ERR_PENDING;
}

static bool ValidatePathSlice(void)
{
    uint32_t budget = PLANNER_VALIDATE_SEGMENT_BUDGET;
    while (budget-- > 0U && s_move.validate_index <= s_move.segments) {
        float progress = Smoothstep((float)s_move.validate_index /
                                    (float)s_move.segments);
        RobotAngles_t angles;
        ErrorCode_t status = Ctrl_Kinematics_Solve(
            s_move.start_x + s_move.delta_x * progress,
            s_move.start_y + s_move.delta_y * progress,
            s_move.start_z + s_move.delta_z * progress, &angles);
        if (status != ERR_OK) {
            s_move.state = PLANNER_IDLE;
            PublishStartResult(status);
            return false;
        }
        s_move.validate_index++;
    }

    if (s_move.validate_index <= s_move.segments) return false;

    s_cur_x = s_move.target_x;
    s_cur_y = s_move.target_y;
    s_cur_z = s_move.target_z;
    Ctrl_MotionEngine_AdjustTheorySteps(s_move.target_m1 - s_move.start_m1,
                                        s_move.target_m2 - s_move.start_m2,
                                        s_move.target_m3 - s_move.start_m3);
    s_move.state = PLANNER_STREAMING;
    PublishStartResult(ERR_OK);
    return true;
}

static bool GenerateOneFrame(void)
{
    if (s_move.generate_index > s_move.segments) {
        s_move.state = PLANNER_IDLE;
        return false;
    }

    const uint32_t index = s_move.generate_index;
    float progress = Smoothstep((float)index / (float)s_move.segments);
    uint32_t cumulative_ticks = (uint32_t)(((uint64_t)s_move.total_ticks * index) /
                                           s_move.segments);
    uint32_t frame_ticks = cumulative_ticks - s_move.previous_tick;
    if (frame_ticks < MIN_FRAME_TICKS) frame_ticks = MIN_FRAME_TICKS;

    RobotAngles_t angles;
    RobotMotorUnits_t units;
    ErrorCode_t status = Ctrl_Kinematics_Solve(
        s_move.start_x + s_move.delta_x * progress,
        s_move.start_y + s_move.delta_y * progress,
        s_move.start_z + s_move.delta_z * progress, &angles);
    if (status != ERR_OK) {
        /* This cannot normally occur after validation.  Stop before a
         * partially generated path can continue if the solver changes. */
        Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_QUEUE_TIMEOUT);
        s_move.state = PLANNER_IDLE;
        return false;
    }
    Ctrl_Kinematics_ToMotorUnits(&angles, &units);

    MotionFrame_t frame = {
        .delta_m1 = units.rot_units - s_move.generated_m1,
        .delta_m2 = units.low_units - s_move.generated_m2,
        .delta_m3 = units.high_units - s_move.generated_m3,
        .total_ticks = frame_ticks,
    };

    if (frame.delta_m1 != 0 || frame.delta_m2 != 0 || frame.delta_m3 != 0) {
        uint32_t steps = RobotMath_MaxAbs3(frame.delta_m1, frame.delta_m2,
                                           frame.delta_m3);
        if (steps + FRAME_STEP_MARGIN > frame.total_ticks)
            frame.total_ticks = steps + FRAME_STEP_MARGIN;
        ApplyStepRateLimit(&frame);

        uint32_t remaining = s_move.segments - index;
        if (remaining < END_SLOW_SEGMENTS)
            frame.total_ticks += (END_SLOW_SEGMENTS - remaining) *
                                 STOP_TAIL_EXTRA_TICKS;
        if (!Ctrl_MotionEngine_PushFrame(&frame)) return false;
    }

    s_move.previous_tick = cumulative_ticks;
    s_move.generated_m1 = units.rot_units;
    s_move.generated_m2 = units.low_units;
    s_move.generated_m3 = units.high_units;
    s_planned_m1 = units.rot_units;
    s_planned_m2 = units.low_units;
    s_planned_m3 = units.high_units;
    s_move.generate_index++;
    if (s_move.generate_index > s_move.segments) s_move.state = PLANNER_IDLE;
    return true;
}

void Ctrl_Planner_Service(void)
{
    if (s_move.state == PLANNER_VALIDATING) {
        if (Ctrl_MotionEngine_HasFault()) {
            s_move.state = PLANNER_IDLE;
            PublishStartResult(ERR_BUSY);
            return;
        }
        (void)ValidatePathSlice();
        return;
    }
    if (s_move.state != PLANNER_STREAMING || Ctrl_MotionEngine_HasFault()) return;

    uint32_t budget = PLANNER_SERVICE_FRAME_BUDGET;
    while (budget-- > 0U && s_move.state == PLANNER_STREAMING) {
        if (!GenerateOneFrame()) return;
    }
}

ErrorCode_t Ctrl_Planner_TeleopStep(float dx, float dy, float dz)
{
    if (Ctrl_MotionEngine_HasFault() || Ctrl_Planner_IsBusy()) return ERR_BUSY;
    if (Ctrl_MotionEngine_GetQueueCount() >= 1U) return ERR_BUSY;

    float tx = s_cur_x + dx;
    float ty = s_cur_y + dy;
    float tz = s_cur_z + dz;

    RobotAngles_t angles;
    RobotMotorUnits_t units;
    ErrorCode_t status = Ctrl_Kinematics_Solve(tx, ty, tz, &angles);
    if (status != ERR_OK) return status;
    Ctrl_Kinematics_ToMotorUnits(&angles, &units);

    MotionFrame_t frame = {
        .delta_m1 = units.rot_units - s_planned_m1,
        .delta_m2 = units.low_units - s_planned_m2,
        .delta_m3 = units.high_units - s_planned_m3,
        .total_ticks = TELEOP_FRAME_MS * TICKS_PER_MS,
    };
    if (frame.delta_m1 == 0 && frame.delta_m2 == 0 && frame.delta_m3 == 0)
        return ERR_OK;

    uint32_t steps = RobotMath_MaxAbs3(frame.delta_m1, frame.delta_m2,
                                        frame.delta_m3);
    if (steps > frame.total_ticks) frame.total_ticks = steps;
    ApplyStepRateLimit(&frame);
    if (!Ctrl_MotionEngine_PushFrame(&frame))
        return Ctrl_MotionEngine_HasFault() ? ERR_BUSY : ERR_BUFFER_FULL;

    Ctrl_MotionEngine_AdjustTheorySteps(frame.delta_m1, frame.delta_m2,
                                        frame.delta_m3);
    s_planned_m1 = units.rot_units;
    s_planned_m2 = units.low_units;
    s_planned_m3 = units.high_units;
    s_cur_x = tx;
    s_cur_y = ty;
    s_cur_z = tz;
    return ERR_OK;
}
