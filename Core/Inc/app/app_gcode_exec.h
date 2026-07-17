/**
 * @file    app_gcode_exec.h
 * @brief   G-code command executor 閳?maps parsed commands to planner calls / G-code 閹稿洣鎶ら幍褑顢戦崳? 鐏忓棜袙閺嬫劗绮ㄩ弸婊勬Ё鐏忓嫪璐熺憴鍕灊閸ｃ劏鐨熼悽?
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
void App_GCodeExec_GetPlannedPosition(float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif

#endif /* __APP_GCODE_EXEC_H__ */
