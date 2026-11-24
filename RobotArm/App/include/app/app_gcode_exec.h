/**
 * @file app_gcode_exec.h
 * @brief G-code 执行适配：委托命令服务，隔离协议会话与业务实现。
 */

#ifndef __APP_GCODE_EXEC_H__
#define __APP_GCODE_EXEC_H__

#include <stdbool.h>

#include "app/app_gcode_parser.h"
#include "error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

ErrorCode_t App_GCodeExec_Init(float start_x, float start_y, float start_z);
ErrorCode_t App_GCodeExec_Run(const GCodeFrame_t *frame);
void App_GCodeExec_Service(void);
bool App_GCodeExec_TakeMoveResult(ErrorCode_t *out_result);
bool App_GCodeExec_TakeQueuedMoveResult(ErrorCode_t *out_result);
bool App_GCodeExec_IsMotionPending(void);
void App_GCodeExec_GetPlannedPosition(float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif

#endif /* __APP_GCODE_EXEC_H__ */
