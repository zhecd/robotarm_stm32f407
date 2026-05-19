/**
 * @file    ctrl_planner.h
 * @brief   Cartesian-to-motor trajectory planner with quintic smoothing / 笛卡尔-电机轨迹规划器 (五次样条平滑)
 * @ingroup control
 *
 * Segments straight-line Cartesian paths into MotionFrame_t chunks.
 * Uses quintic (5th-order) smoothstep velocity profiling for jerk-limited
 * acceleration and deceleration.
 * 将笛卡尔直线路径分段为 MotionFrame_t 帧, 使用五次平滑阶跃速度曲线实现加加速度受限的加减速。
 */

#ifndef __CTRL_PLANNER_H__
#define __CTRL_PLANNER_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Ctrl_Planner_Init(float start_x, float start_y, float start_z);

/** Plan and queue a straight-line move / 规划并排队直线运动 */
bool Ctrl_Planner_MoveLine(float target_x, float target_y, float target_z, uint32_t duration_ms);

/** Single step for teleop joystick control / 遥控摇杆单步移动 */
bool Ctrl_Planner_TeleopStep(float dx, float dy, float dz);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_PLANNER_H__ */
