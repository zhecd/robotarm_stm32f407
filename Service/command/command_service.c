#include "command_service.h"

#include "platform_critical.h"
#include "robot_config.h"
#include "safety_service.h"
#include "service/control/ctrl_planner.h"
#include "service/svc_gripper.h"

#include <math.h>

static CommandServiceStatus_t s_status = {
    .active_source = COMMAND_SOURCE_GCODE,
    .feedrate_mm_min = GCODE_DEFAULT_FEEDRATE,
};

typedef struct {
    bool active;
    float target_x_mm;
    float target_y_mm;
    float target_z_mm;
    float feedrate_mm_min;
} PendingMove_t;

static PendingMove_t s_pending_move;
static bool s_move_result_ready;
static ErrorCode_t s_move_result;

static uint32_t ComputeDuration(float target_x, float target_y, float target_z,
                                float feedrate_mm_min)
{
    float dx = target_x - s_status.planned_x_mm;
    float dy = target_y - s_status.planned_y_mm;
    float dz = target_z - s_status.planned_z_mm;
    float distance = sqrtf(dx * dx + dy * dy + dz * dz);
    if (distance <= GCODE_MIN_MOVE_MM) return 0U;
    uint32_t duration = (uint32_t)((distance * 60000.0f) / feedrate_mm_min);
    return duration == 0U ? GCODE_MIN_DURATION_MS : duration;
}

ErrorCode_t CommandService_Init(float start_x, float start_y, float start_z)
{
    ErrorCode_t result = Ctrl_Planner_Init(start_x, start_y, start_z);
    if (result != ERR_OK) return result;

    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_status.generation++;
    s_status.active_source = COMMAND_SOURCE_GCODE;
    s_status.planned_x_mm = start_x;
    s_status.planned_y_mm = start_y;
    s_status.planned_z_mm = start_z;
    s_status.feedrate_mm_min = GCODE_DEFAULT_FEEDRATE;
    PlatformCritical_Exit(state);
    s_pending_move.active = false;
    s_move_result_ready = false;
    s_move_result = ERR_OK;
    return ERR_OK;
}

static void CommitMove(float target_x, float target_y, float target_z, float feedrate)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_status.planned_x_mm = target_x;
    s_status.planned_y_mm = target_y;
    s_status.planned_z_mm = target_z;
    s_status.feedrate_mm_min = feedrate;
    s_status.active_source = COMMAND_SOURCE_GCODE;
    s_status.generation++;
    PlatformCritical_Exit(state);
}

void CommandService_Service(void)
{
    ErrorCode_t result;
    Ctrl_Planner_Service();
    if (!s_pending_move.active || !Ctrl_Planner_TakeStartResult(&result)) return;

    if (result == ERR_OK) {
        CommitMove(s_pending_move.target_x_mm, s_pending_move.target_y_mm,
                   s_pending_move.target_z_mm, s_pending_move.feedrate_mm_min);
    }
    s_pending_move.active = false;
    s_move_result = result;
    s_move_result_ready = true;
}

bool CommandService_IsMotionPending(void)
{
    return Ctrl_Planner_IsBusy();
}

bool CommandService_TakeMoveResult(ErrorCode_t *out_result)
{
    if (!s_move_result_ready) return false;
    if (out_result) *out_result = s_move_result;
    s_move_result_ready = false;
    return true;
}

ErrorCode_t CommandService_RunGCode(const GCodeFrame_t *frame)
{
    if (!frame) return ERR_NULL_PARAM;
    if (!SafetyService_IsMotionAllowed() && (frame->type == GCMD_G0 || frame->type == GCMD_G1))
        return ERR_BUSY;

    switch (frame->type) {
    case GCMD_G0:
    case GCMD_G1: {
        if (s_pending_move.active) return ERR_BUSY;
        float target_x = frame->has_x ? frame->x : s_status.planned_x_mm;
        float target_y = frame->has_y ? frame->y : s_status.planned_y_mm;
        float target_z = frame->has_z ? frame->z : s_status.planned_z_mm;
        if (frame->has_f && ((float)frame->f > GCODE_MAX_FEEDRATE || frame->f == 0U))
            return ERR_OUT_OF_RANGE;
        float feedrate = frame->has_f ? (float)frame->f : s_status.feedrate_mm_min;

        ErrorCode_t result = Ctrl_Planner_MoveLine(target_x, target_y, target_z,
                                                    ComputeDuration(target_x, target_y, target_z,
                                                                    feedrate));
        if (result == ERR_PENDING) {
            s_pending_move = (PendingMove_t){
                .active = true,
                .target_x_mm = target_x,
                .target_y_mm = target_y,
                .target_z_mm = target_z,
                .feedrate_mm_min = feedrate,
            };
            return ERR_PENDING;
        }
        if (result != ERR_OK) return result;
        CommitMove(target_x, target_y, target_z, feedrate);
        return ERR_OK;
    }
    case GCMD_M3:
        Svc_Gripper_Open();
        return ERR_OK;
    case GCMD_M5:
        Svc_Gripper_Close();
        return ERR_OK;
    default:
        return ERR_OUT_OF_RANGE;
    }
}

ErrorCode_t CommandService_RunTeleopStep(float dx_mm, float dy_mm, float dz_mm)
{
    if (!SafetyService_IsMotionAllowed() || s_pending_move.active) return ERR_BUSY;

    ErrorCode_t result = Ctrl_Planner_TeleopStep(dx_mm, dy_mm, dz_mm);
    if (result != ERR_OK) return result;

    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_status.planned_x_mm += dx_mm;
    s_status.planned_y_mm += dy_mm;
    s_status.planned_z_mm += dz_mm;
    s_status.active_source = COMMAND_SOURCE_PS2;
    s_status.generation++;
    PlatformCritical_Exit(state);
    return ERR_OK;
}

void CommandService_GetStatus(CommandServiceStatus_t *out_status)
{
    if (!out_status) return;
    PlatformCriticalState_t state = PlatformCritical_Enter();
    *out_status = s_status;
    PlatformCritical_Exit(state);
}
