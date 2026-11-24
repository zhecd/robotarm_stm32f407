/**
 * @file command_service.c
 * @brief 命令服务：来源仲裁、缺省坐标继承、路径点 FIFO 与异步结果。
 */
#include "command_service.h"

#include "platform_critical.h"
#include "robot_config.h"
#include "motion_service.h"
#include "planning_service.h"
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

static bool ComputeDuration(float start_x, float start_y, float start_z,
                                float target_x, float target_y, float target_z,
                                float feedrate_mm_min, uint32_t *out_ms)
{
    double dx = (double)target_x - start_x;
    double dy = (double)target_y - start_y;
    double dz = (double)target_z - start_z;
    double distance = sqrt(dx*dx + dy*dy + dz*dz);
    if (!isfinite(distance) || !isfinite(feedrate_mm_min) || feedrate_mm_min <= 0) return false;
    double duration = distance * 60000.0 / feedrate_mm_min;
    if (!isfinite(duration) || duration > UINT32_MAX) return false;
    *out_ms = distance <= GCODE_MIN_MOVE_MM ? 0U :
              duration < 1.0 ? GCODE_MIN_DURATION_MS : (uint32_t)duration;
    return true;
}

ErrorCode_t CommandService_Init(float start_x, float start_y, float start_z)
{
    ErrorCode_t result = PlanningService_Init(start_x, start_y, start_z);
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
    uint32_t duration_ms;
    if (!ComputeDuration(s_status.planned_x_mm, s_status.planned_y_mm,
                         s_status.planned_z_mm, move->target_x_mm, move->target_y_mm,
                         move->target_z_mm, move->feedrate_mm_min, &duration_ms)) return ERR_OUT_OF_RANGE;
    ErrorCode_t result = PlanningService_StartLine(
        move->target_x_mm, move->target_y_mm, move->target_z_mm,
        duration_ms);
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

/** 主循环按预算推进规划；故障时取消待处理请求，禁止残留命令继续生成运动帧。 */
void CommandService_Service(void)
{
    if (!MotionService_IsActionAllowed()) {
        CommandService_AbortPending();
        return;
    }
    ErrorCode_t result;
    PlanningService_Service();
    if (s_pending_move.active && PlanningService_TakeStartResult(&result)) {
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

    if (s_pending_move.active || PlanningService_IsBusy()) return;

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
    return s_pending_move.active || PlanningService_IsBusy() || WaypointQueueCount() > 0U;
}

void CommandService_AbortPending(void)
{
    PlanningService_Cancel();
    s_pending_move.active = false;
    s_move_result_ready = false;
    s_queued_move_result_ready = false;
    WaypointQueueClearAndRebase();
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

/** ERR_PENDING=等待路径校验，ERR_QUEUED=已入坐标 FIFO；二者都不表示动作完成。 */
ErrorCode_t CommandService_RunGCode(const GCodeFrame_t *frame)
{
    if (!frame) return ERR_NULL_PARAM;
    bool action=frame->type==GCMD_G0 || frame->type==GCMD_G1 ||
                frame->type==GCMD_M3 || frame->type==GCMD_M5;
    /* 夹爪和移动共用启动/故障许可；Motion尚未映射给Safety时也不能接受动作。 */
    if (action && !MotionService_IsActionAllowed())
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
        if (!isfinite(target_x) || !isfinite(target_y) || !isfinite(target_z)) return ERR_OUT_OF_RANGE;
        if (frame->has_f && ((float)frame->f > GCODE_MAX_FEEDRATE || frame->f == 0U))
            return ERR_OUT_OF_RANGE;
        float feedrate = frame->has_f ? (float)frame->f : s_enqueue_feedrate_mm_min;
        QueuedMove_t move = {
            .target_x_mm = target_x,
            .target_y_mm = target_y,
            .target_z_mm = target_z,
            .feedrate_mm_min = feedrate,
        };

        if (s_pending_move.active || PlanningService_IsBusy() || WaypointQueueCount() > 0U) {
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
        return Svc_Gripper_Open() ? ERR_OK : ERR_BUSY;
    case GCMD_M5:
        return Svc_Gripper_Close() ? ERR_OK : ERR_BUSY;
    default:
        return ERR_OUT_OF_RANGE;
    }
}

/** 输入dx/dy/dz为mm；主循环单生产者调用。
 * PS2自己的当前帧可继续运行，规划层最多再接受一个等待帧，避免每10ms轮询
 * 都等待ISR完全排空造成段间空转。G-code和闭环恢复不得混入这个短帧管线。
 * 松杆不丢弃已受理终点；失联/故障仍立即撤销当前帧和等待帧。 */
ErrorCode_t CommandService_RunTeleopStep(float dx_mm, float dy_mm, float dz_mm)
{
    if (!MotionService_IsActionAllowed() || s_pending_move.active ||
        WaypointQueueCount() > 0U || MotionService_IsClosedLoopRecoveryActive())
        return ERR_BUSY;
    if (!MotionService_IsIdle() && s_status.active_source != COMMAND_SOURCE_PS2)
        return ERR_BUSY;

    ErrorCode_t result = PlanningService_StartTeleopStep(dx_mm, dy_mm, dz_mm);
    if (result != ERR_OK) return result;

    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_status.planned_x_mm += dx_mm;
    s_status.planned_y_mm += dy_mm;
    s_status.planned_z_mm += dz_mm;
    s_status.active_source = COMMAND_SOURCE_PS2;
    s_status.generation++;
    PlatformCritical_Exit(state);
    /* PS2与G-code共享坐标：这里已确认没有待处理串口路径点。
     * 连同缺省轴/进给继承缓存一起同步，切回后的G1 X...才会保留最新Y/Z。 */
    WaypointQueueClearAndRebase();
    /* 遥操作也同步闭环目标到已受理理论步数，避免切回G-code后追逐旧目标。 */
    MotionService_SyncClosedLoopTarget();
    if (MotionService_HasFault()) return ERR_BUSY;
    return ERR_OK;
}

void CommandService_GetStatus(CommandServiceStatus_t *out_status)
{
    if (!out_status) return;
    PlatformCriticalState_t state = PlatformCritical_Enter();
    *out_status = s_status;
    PlatformCritical_Exit(state);
}
