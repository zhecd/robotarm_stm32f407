/**
 * @file command_service.h
 * @brief 命令服务：来源仲裁、缺省坐标继承、路径点 FIFO 与异步结果。
 */
#ifndef ROBOTARM_COMMAND_SERVICE_H
#define ROBOTARM_COMMAND_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "command_types.h"
#include "common/error_code.h"

typedef enum {
    COMMAND_SOURCE_GCODE = 0,
    COMMAND_SOURCE_PS2
} CommandSource_t;

typedef struct {
    uint32_t generation;
    CommandSource_t active_source;
    float planned_x_mm;
    float planned_y_mm;
    float planned_z_mm;
    float feedrate_mm_min;
} CommandServiceStatus_t;

ErrorCode_t CommandService_Init(float start_x, float start_y, float start_z);
/** ERR_PENDING=等待路径校验，ERR_QUEUED=已入坐标 FIFO；二者都不表示动作完成。 */
ErrorCode_t CommandService_RunGCode(const GCodeFrame_t *frame);
/** 输入增量单位mm；PS2最多当前帧+一个等待帧。ERR_BUSY不推进目标。
 * 仅主循环调用；保留安全许可、G-code隔离和闭环恢复门禁。 */
ErrorCode_t CommandService_RunTeleopStep(float dx_mm, float dy_mm, float dz_mm);
/** 主循环按预算推进规划；故障时取消待处理请求，禁止残留命令继续生成运动帧。 */
void CommandService_Service(void);
/** 撤销路径点、待校验请求和未读取结果；重新回零后方可继续运动。 */
void CommandService_AbortPending(void);
bool CommandService_TakeMoveResult(ErrorCode_t *out_result);
bool CommandService_TakeQueuedMoveResult(ErrorCode_t *out_result);
bool CommandService_IsMotionPending(void);
uint16_t CommandService_GetQueuedMoveCount(void);
void CommandService_GetStatus(CommandServiceStatus_t *out_status);

#endif
