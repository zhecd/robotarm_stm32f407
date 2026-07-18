#include "command_service.h"

#include "platform_critical.h"
#include "robot_config.h"
#include "safety_service.h"
#include "motion_service.h"
#include "ctrl_planner.h"
#include "service/svc_gripper.h"

#include <math.h>

static CommandServiceStatus_t s_status = {
    .active_source = COMMAND_SOURCE_GCODE,
    .feedrate_mm_min = GCODE_DEFAULT_FEEDRATE,
};

typedef struct {
    bool active;
    bool foreground;
    float target_x_mm;
    float target_y_mm;
    float target_z_mm;
    float feedrate_mm_min;
} PendingMove_t;

typedef struct {
    float target_x_mm;
    float target_y_mm;
    float target_z_mm;
    float feedrate_mm_min;
} QueuedMove_t;

static PendingMove_t s_pending_move;
static bool s_move_result_ready;
static ErrorCode_t s_move_result;
/* The ring keeps one storage slot empty to distinguish full from empty.
   Allocate one additional slot so COMMAND_WAYPOINT_QUEUE_SIZE remains the
   documented usable capacity. */
static QueuedMove_t s_waypoint_queue[COMMAND_WAYPOINT_QUEUE_SIZE + 1U];
static uint16_t s_waypoint_head;
static uint16_t s_waypoint_tail;
static float s_enqueue_x_mm;
static float s_enqueue_y_mm;
static float s_enqueue_z_mm;
static float s_enqueue_feedrate_mm_min;
static bool s_queued_move_result_ready;
static ErrorCode_t s_queued_move_result;

static uint16_t WaypointQueueCount(void)
{
    return (s_waypoint_head >= s_waypoint_tail)
         ? (uint16_t)(s_waypoint_head - s_waypoint_tail)
         : (uint16_t)((COMMAND_WAYPOINT_QUEUE_SIZE + 1U) - s_waypoint_tail + s_waypoint_head);
}

static bool WaypointQueuePush(const QueuedMove_t *move)
{
    uint16_t next = (uint16_t)((s_waypoint_head + 1U) % (COMMAND_WAYPOINT_QUEUE_SIZE + 1U));
    if (next == s_waypoint_tail) return false;
    s_waypoint_queue[s_waypoint_head] = *move;
    s_waypoint_head = next;
    return true;
}

static bool WaypointQueuePop(QueuedMove_t *move)
{
    if (s_waypoint_head == s_waypoint_tail || !move) return false;
    *move = s_waypoint_queue[s_waypoint_tail];
    s_waypoint_tail = (uint16_t)((s_waypoint_tail + 1U) % (COMMAND_WAYPOINT_QUEUE_SIZE + 1U));
    return true;
}

static void WaypointQueueClearAndRebase(void)
{
    s_waypoint_tail = s_waypoint_head;
    s_enqueue_x_mm = s_status.planned_x_mm;
    s_enqueue_y_mm = s_status.planned_y_mm;
    s_enqueue_z_mm = s_status.planned_z_mm;
    s_enqueue_feedrate_mm_min = s_status.feedrate_mm_min;
}

static uint32_t ComputeDuration(float start_x, float start_y, float start_z,
                                float target_x, float target_y, float target_z,
                                float feedrate_mm_min)
{
    float dx = target_x - start_x;
    float dy = target_y - start_y;
    float dz = target_z - start_z;
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
    s_waypoint_head = 0U;
    s_waypoint_tail = 0U;
    s_enqueue_x_mm = start_x;
    s_enqueue_y_mm = start_y;
    s_enqueue_z_mm = start_z;
    s_enqueue_feedrate_mm_min = GCODE_DEFAULT_FEEDRATE;
    s_queued_move_result_ready = false;
    s_queued_move_result = ERR_OK;
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

static ErrorCode_t StartMove(const QueuedMove_t *move, bool foreground)
{
    ErrorCode_t result = Ctrl_Planner_MoveLine(
        move->target_x_mm, move->target_y_mm, move->target_z_mm,
        ComputeDuration(s_status.planned_x_mm, s_status.planned_y_mm,
                        s_status.planned_z_mm, move->target_x_mm,
                        move->target_y_mm, move->target_z_mm,
                        move->feedrate_mm_min));
    if (result == ERR_PENDING) {
        s_pending_move = (PendingMove_t){
            .active = true,
            .foreground = foreground,
            .target_x_mm = move->target_x_mm,
            .target_y_mm = move->target_y_mm,
            .target_z_mm = move->target_z_mm,
            .feedrate_mm_min = move->feedrate_mm_min,
        };
    } else if (result == ERR_OK) {
        CommitMove(move->target_x_mm, move->target_y_mm, move->target_z_mm,
                   move->feedrate_mm_min);
    }
    return result;
}

void CommandService_Service(void)
{
    ErrorCode_t result;
    Ctrl_Planner_Service();
    if (s_pending_move.active && Ctrl_Planner_TakeStartResult(&result)) {
        bool foreground = s_pending_move.foreground;
        if (result == ERR_OK) {
            CommitMove(s_pending_move.target_x_mm, s_pending_move.target_y_mm,
                       s_pending_move.target_z_mm, s_pending_move.feedrate_mm_min);
        } else {
            /* A path validation failure invalidates all later absolute
               waypoints, whose reference pose would no longer be valid. */
            WaypointQueueClearAndRebase();
        }
        s_pending_move.active = false;
        if (foreground) {
            s_move_result = result;
            s_move_result_ready = true;
        } else {
            s_queued_move_result = result;
            s_queued_move_result_ready = true;
        }
        return;
    }

    if (s_pending_move.active || Ctrl_Planner_IsBusy()) return;

    QueuedMove_t queued;
    if (!WaypointQueuePop(&queued)) return;
    result = StartMove(&queued, false);
    if (result == ERR_OK) {
        s_queued_move_result = ERR_OK;
        s_queued_move_result_ready = true;
    } else if (result != ERR_PENDING) {
        WaypointQueueClearAndRebase();
        s_queued_move_result = result;
        s_queued_move_result_ready = true;
    }
}

bool CommandService_IsMotionPending(void)
{
    return s_pending_move.active || Ctrl_Planner_IsBusy() || WaypointQueueCount() > 0U;
}

bool CommandService_TakeMoveResult(ErrorCode_t *out_result)
{
    if (!s_move_result_ready) return false;
    if (out_result) *out_result = s_move_result;
    s_move_result_ready = false;
    return true;
}

bool CommandService_TakeQueuedMoveResult(ErrorCode_t *out_result)
{
    if (!s_queued_move_result_ready) return false;
    if (out_result) *out_result = s_queued_move_result;
    s_queued_move_result_ready = false;
    return true;
}

uint16_t CommandService_GetQueuedMoveCount(void)
{
    return WaypointQueueCount();
}

ErrorCode_t CommandService_RunGCode(const GCodeFrame_t *frame)
{
    if (!frame) return ERR_NULL_PARAM;
    if (!SafetyService_IsMotionAllowed() && (frame->type == GCMD_G0 || frame->type == GCMD_G1))
        return ERR_BUSY;
    if (MotionService_IsClosedLoopRecoveryActive() &&
        (frame->type == GCMD_G0 || frame->type == GCMD_G1))
        return ERR_BUSY;

    switch (frame->type) {
    case GCMD_G0:
    case GCMD_G1: {
        float target_x = frame->has_x ? frame->x : s_enqueue_x_mm;
        float target_y = frame->has_y ? frame->y : s_enqueue_y_mm;
        float target_z = frame->has_z ? frame->z : s_enqueue_z_mm;
        if (frame->has_f && ((float)frame->f > GCODE_MAX_FEEDRATE || frame->f == 0U))
            return ERR_OUT_OF_RANGE;
        float feedrate = frame->has_f ? (float)frame->f : s_enqueue_feedrate_mm_min;
        QueuedMove_t move = {
            .target_x_mm = target_x,
            .target_y_mm = target_y,
            .target_z_mm = target_z,
            .feedrate_mm_min = feedrate,
        };

        if (s_pending_move.active || Ctrl_Planner_IsBusy() || WaypointQueueCount() > 0U) {
            if (!WaypointQueuePush(&move)) return ERR_BUFFER_FULL;
            s_enqueue_x_mm = target_x;
            s_enqueue_y_mm = target_y;
            s_enqueue_z_mm = target_z;
            s_enqueue_feedrate_mm_min = feedrate;
            return ERR_QUEUED;
        }

        ErrorCode_t result = StartMove(&move, true);
        if (result == ERR_PENDING || result == ERR_OK) {
            s_enqueue_x_mm = target_x;
            s_enqueue_y_mm = target_y;
            s_enqueue_z_mm = target_z;
            s_enqueue_feedrate_mm_min = feedrate;
        }
        return result;
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
    if (!SafetyService_IsMotionAllowed() || s_pending_move.active ||
        WaypointQueueCount() > 0U || MotionService_IsClosedLoopRecoveryActive())
        return ERR_BUSY;

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
