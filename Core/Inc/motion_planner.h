#ifndef __MOTION_PLANNER_H__
#define __MOTION_PLANNER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ==============================================================
// 运动规划层 API 声明 (大脑皮层)
// ==============================================================

/**
 * @brief  初始化运动规划器
 */
void Motion_Planner_Init(float start_x, float start_y, float start_z);

/**
 * @brief  笛卡尔空间直线插补 (画绝对直线，带S型加减速，长队列阻塞)
 */
bool Motion_Planner_MoveLine(float target_x, float target_y, float target_z, uint32_t duration_ms);

/**
 * @brief  [新增] PS2手柄专用：非阻塞微步进 (无曲线，短平快)
 * @param  dx, dy, dz: 当前想要移动的微小增量 (毫米)
 */
bool Motion_Planner_TeleopStep(float dx, float dy, float dz);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_PLANNER_H__ */