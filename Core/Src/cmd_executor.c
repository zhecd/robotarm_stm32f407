#include "cmd_executor.h"
#include "bsp_gripper.h"
#include "motion_planner.h"
#include <math.h>

#define CMD_EXECUTOR_DEFAULT_FEEDRATE_MM_MIN 3000.0f
#define CMD_EXECUTOR_MIN_MOVE_DISTANCE_MM    0.001f
#define CMD_EXECUTOR_MIN_DURATION_MS         1U

typedef struct
{
    float current_x;
    float current_y;
    float current_z;
    float current_feedrate;
} CmdExecutorState_t;

static CmdExecutorState_t s_cmd_executor = {
    .current_x = 0.0f,
    .current_y = 0.0f,
    .current_z = 0.0f,
    .current_feedrate = CMD_EXECUTOR_DEFAULT_FEEDRATE_MM_MIN
};

static uint32_t Cmd_Executor_ComputeDurationMs(float target_x, float target_y, float target_z)
{
    float dx = target_x - s_cmd_executor.current_x;
    float dy = target_y - s_cmd_executor.current_y;
    float dz = target_z - s_cmd_executor.current_z;
    float distance = sqrtf((dx * dx) + (dy * dy) + (dz * dz));

    if (distance <= CMD_EXECUTOR_MIN_MOVE_DISTANCE_MM) {
        return 0U;
    }

    uint32_t duration_ms = (uint32_t)((distance * 60000.0f) / s_cmd_executor.current_feedrate);
    return (duration_ms == 0U) ? CMD_EXECUTOR_MIN_DURATION_MS : duration_ms;
}

static void Cmd_Executor_RunLinearMove(const GCodeFrame_t *frame)
{
    float target_x = frame->has_x ? frame->x : s_cmd_executor.current_x;
    float target_y = frame->has_y ? frame->y : s_cmd_executor.current_y;
    float target_z = frame->has_z ? frame->z : s_cmd_executor.current_z;

    if (frame->has_f && (frame->f > 0U)) {
        s_cmd_executor.current_feedrate = (float)frame->f;
    }

    uint32_t duration_ms = Cmd_Executor_ComputeDurationMs(target_x, target_y, target_z);
    Motion_Planner_MoveLine(target_x, target_y, target_z, duration_ms);

    s_cmd_executor.current_x = target_x;
    s_cmd_executor.current_y = target_y;
    s_cmd_executor.current_z = target_z;
}

void Cmd_Executor_Init(float start_x, float start_y, float start_z)
{
    s_cmd_executor.current_x = start_x;
    s_cmd_executor.current_y = start_y;
    s_cmd_executor.current_z = start_z;
    s_cmd_executor.current_feedrate = CMD_EXECUTOR_DEFAULT_FEEDRATE_MM_MIN;
}

void Cmd_Executor_Run(const GCodeFrame_t *frame)
{
    if (frame == NULL) {
        return;
    }

    switch (frame->type) {
    case GCMD_G0:
    case GCMD_G1:
        Cmd_Executor_RunLinearMove(frame);
        break;

    case GCMD_M3:
        BSP_Gripper_Open(&hgripper);
        break;

    case GCMD_M5:
        BSP_Gripper_Close(&hgripper);
        break;

    case GCMD_UNKNOWN:
    default:
        break;
    }
}
