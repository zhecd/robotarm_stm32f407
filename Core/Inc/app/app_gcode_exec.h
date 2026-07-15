/**
 * @file    app_gcode_exec.h
 * @brief   G-code command executor — maps parsed commands to planner calls / G-code 指令执行器, 将解析结果映射为规划器调用
 * @ingroup app
 */

#ifndef __APP_GCODE_EXEC_H__
#define __APP_GCODE_EXEC_H__

#include "app/app_gcode_parser.h"
#include "error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

void App_GCodeExec_Init(float start_x, float start_y, float start_z);
ErrorCode_t App_GCodeExec_Run(const GCodeFrame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __APP_GCODE_EXEC_H__ */
