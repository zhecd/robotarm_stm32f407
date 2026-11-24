/**
 * @file ctrl_planner.h
 * @brief 规划内部实现：先校验整条路径，再按预算逐步生成 STEP 运动帧。
 * @details 五次平滑函数改善段间起停，但代码检查不证明整机加速度或抖动已实测达标。
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

/* Plan and queue a straight-line move */
/* Returns ERR_PENDING for a nonzero-length move.  Path validation and frame
 * generation are advanced by Ctrl_Planner_Service() without blocking. */
ErrorCode_t Ctrl_Planner_MoveLine(float target_x, float target_y, float target_z, uint32_t duration_ms);

/* Single step for teleop joystick control */
ErrorCode_t Ctrl_Planner_TeleopStep(float dx, float dy, float dz);
void Ctrl_Planner_Service(void);
void Ctrl_Planner_Cancel(void);
bool Ctrl_Planner_IsBusy(void);
bool Ctrl_Planner_TakeStartResult(ErrorCode_t *out_result);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_PLANNER_H__ */
