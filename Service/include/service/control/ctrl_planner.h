/**
 * @file    ctrl_planner.h
 * @brief   Cartesian-to-motor trajectory planner with quintic smoothing / 绗涘崱灏?鐢垫満杞ㄨ抗瑙勫垝�?(浜旀鏍锋潯骞虫�?
 * @ingroup control
 *
 * Segments straight-line Cartesian paths into MotionFrame_t chunks.
 * Uses quintic (5th-order) smoothstep velocity profiling for jerk-limited
 * acceleration and deceleration.
 * 灏嗙瑳鍗″皵鐩寸嚎璺緞鍒嗘涓?MotionFrame_t �? 浣跨敤浜旀骞虫粦闃惰穬閫熷害鏇茬嚎瀹炵幇鍔犲姞閫熷害鍙楅檺鐨勫姞鍑忛€熴€?
 */

#ifndef __CTRL_PLANNER_H__
#define __CTRL_PLANNER_H__

#include <stdbool.h>
#include <stdint.h>
#include "error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

ErrorCode_t Ctrl_Planner_Init(float start_x, float start_y, float start_z);

/** Plan and queue a straight-line move / 瑙勫垝骞舵帓闃熺洿绾胯繍�?*/
/* Returns ERR_PENDING for a nonzero-length move.  Path validation and frame
 * generation are advanced by Ctrl_Planner_Service() without blocking. */
ErrorCode_t Ctrl_Planner_MoveLine(float target_x, float target_y, float target_z, uint32_t duration_ms);

/** Single step for teleop joystick control / 閬ユ帶鎽囨潌鍗曟绉诲姩 */
ErrorCode_t Ctrl_Planner_TeleopStep(float dx, float dy, float dz);
void Ctrl_Planner_Service(void);
bool Ctrl_Planner_IsBusy(void);
bool Ctrl_Planner_TakeStartResult(ErrorCode_t *out_result);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_PLANNER_H__ */
