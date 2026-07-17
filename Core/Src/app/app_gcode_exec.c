/**
 * @file    app_gcode_exec.c
 * @brief   G-code command executor implementation. / G-code 指令执行器实现。
 * @ingroup app
 */

#include "app/app_gcode_exec.h"
#include "control/ctrl_gripper.h"
#include "control/ctrl_planner.h"
#include "robot_config.h"
#include <math.h>

typedef struct {
    float cur_x, cur_y, cur_z;
    float feedrate;
} ExecState_t;

static ExecState_t s_exec = {
    .cur_x = 0.0f, .cur_y = 0.0f, .cur_z = 0.0f,
    .feedrate = GCODE_DEFAULT_FEEDRATE
};

static uint32_t ComputeDuration(float tx, float ty, float tz)
{
    float dx = tx - s_exec.cur_x;
    float dy = ty - s_exec.cur_y;
    float dz = tz - s_exec.cur_z;
    float dist = sqrtf(dx * dx + dy * dy + dz * dz);
    if (dist <= GCODE_MIN_MOVE_MM) return 0U;

    uint32_t ms = (uint32_t)((dist * 60000.0f) / s_exec.feedrate);
    return (ms == 0U) ? GCODE_MIN_DURATION_MS : ms;
}

static ErrorCode_t RunLinearMove(const GCodeFrame_t *frame)
{
    float tx = frame->has_x ? frame->x : s_exec.cur_x;
    float ty = frame->has_y ? frame->y : s_exec.cur_y;
    float tz = frame->has_z ? frame->z : s_exec.cur_z;

    if (frame->has_f && (float)frame->f > GCODE_MAX_FEEDRATE)
        return ERR_OUT_OF_RANGE;
    if (frame->has_f && frame->f > 0U)
        s_exec.feedrate = (float)frame->f;

    uint32_t dur = ComputeDuration(tx, ty, tz);
    ErrorCode_t status = Ctrl_Planner_MoveLine(tx, ty, tz, dur);
    if (status != ERR_OK) return status;

    s_exec.cur_x = tx;
    s_exec.cur_y = ty;
    s_exec.cur_z = tz;
    return ERR_OK;
}

void App_GCodeExec_Init(float sx, float sy, float sz)
{
    s_exec.cur_x = sx;
    s_exec.cur_y = sy;
    s_exec.cur_z = sz;
    s_exec.feedrate = GCODE_DEFAULT_FEEDRATE;
}

void App_GCodeExec_GetPlannedPosition(float *x, float *y, float *z)
{
    if (x) *x = s_exec.cur_x;
    if (y) *y = s_exec.cur_y;
    if (z) *z = s_exec.cur_z;
}

ErrorCode_t App_GCodeExec_Run(const GCodeFrame_t *frame)
{
    if (!frame) return ERR_NULL_PARAM;

    switch (frame->type) {
    case GCMD_G0:
    case GCMD_G1:
        return RunLinearMove(frame);
    case GCMD_M3:
        Ctrl_Gripper_Open();
        return ERR_OK;
    case GCMD_M5:
        Ctrl_Gripper_Close();
        return ERR_OK;
    case GCMD_UNKNOWN:
    default:
        return ERR_OUT_OF_RANGE;
    }
}
