/**
 * @file    ctrl_closed_loop.h
 * @brief   3-axis PID closed-loop position controller / 三轴 PID 闭环位置控制器
 * @ingroup control
 *
 * Runs at 50 Hz. Compares encoder readings against theory-step targets
 * and injects correction frames into the motion buffer.
 * 以 50Hz 运行, 将编码器读数与理论步数目标比较, 向运动缓冲注入修正帧。
 *
 * Features / 特性:
 *   - Adaptive gain (conservative for small errors, aggressive for large) / 自适应增益 (小误差保守, 大误差强力)
 *   - EMA low-pass filter on encoder readings / 编码器 EMA 低通滤波
 *   - Integral separation to prevent windup / 积分分离防饱和
 *   - Stuck-encoder detection with automatic axis disable / 编码器卡死检测自动禁用轴
 *   - Cooldown timer between corrections / 修正间冷却计时
 */

#ifndef __CTRL_CLOSED_LOOP_H__
#define __CTRL_CLOSED_LOOP_H__

#include <stdbool.h>
#include <stdint.h>
#include "common/error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CL_AXIS_COUNT  3     /* Number of controlled axes / 受控轴数 */

void Ctrl_ClosedLoop_Init(void);

/** Sync PID targets from current theory-step positions / 从当前理论步数位置同步 PID 目标 */
void Ctrl_ClosedLoop_SyncTarget(void);

/** Run one PID iteration (call at 50 Hz) / 运行一次 PID 迭代 (50Hz 调用) */
void Ctrl_ClosedLoop_Update(void);

/** Per-axis state queries / 逐轴状态查询 */
bool          Ctrl_ClosedLoop_IsAxisEnabled(int axis);
void          Ctrl_ClosedLoop_SetAxisEnabled(int axis, bool en);

/** Get latest encoder angle for an axis (deg), with multi-turn update.
    成功返回 true 并写入 *out_deg, 失败返回 false 且 *out_deg=0. */
bool          Ctrl_ClosedLoop_GetAxisAngle(int axis, float *out_deg);

/** Set zero point for an axis encoder / 设置轴编码器零点, 成功返回 ERR_OK */
ErrorCode_t   Ctrl_ClosedLoop_SetAxisZero(int axis);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_CLOSED_LOOP_H__ */
