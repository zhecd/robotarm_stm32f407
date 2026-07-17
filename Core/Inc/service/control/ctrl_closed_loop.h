/**
 * @file    ctrl_closed_loop.h
 * @brief   3-axis PID closed-loop position controller / 涓夎�?PID 闂幆浣嶇疆鎺у埗�?
 * @ingroup control
 *
 * Runs at 50 Hz. Compares encoder readings against theory-step targets
 * and injects correction frames into the motion buffer.
 * �?50Hz 杩愯�? 灏嗙紪鐮佸櫒璇绘暟涓庣悊璁烘鏁扮洰鏍囨瘮杈? 鍚戣繍鍔ㄧ紦鍐叉敞鍏ヤ慨姝ｅ抚銆?
 *
 * Features / 鐗规�?
 *   - Adaptive gain (conservative for small errors, aggressive for large) / 鑷€傚簲澧炵泭 (灏忚宸繚�? 澶ц宸己鍔?
 *   - EMA low-pass filter on encoder readings / 缂栫爜鍣?EMA 浣庨€氭护娉?
 *   - Integral separation to prevent windup / 绉垎鍒嗙闃查ケ鍜?
 *   - Stuck-encoder detection with automatic axis disable / 缂栫爜鍣ㄥ崱姝绘娴嬭嚜鍔ㄧ鐢ㄨ酱
 *   - Cooldown timer between corrections / 淇闂村喎鍗磋鏃?
 */

#ifndef __CTRL_CLOSED_LOOP_H__
#define __CTRL_CLOSED_LOOP_H__

#include <stdbool.h>
#include <stdint.h>
#include "common/error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CL_AXIS_COUNT  3     /* Number of controlled axes / 鍙楁帶杞存暟 */

void Ctrl_ClosedLoop_Init(void);

/** Sync PID targets from current theory-step positions / 浠庡綋鍓嶇悊璁烘鏁颁綅缃悓姝?PID 鐩�?*/
void Ctrl_ClosedLoop_SyncTarget(void);

/** Run one PID iteration (call at 50 Hz) / 杩愯涓€娆?PID 杩�?(50Hz 璋冪�? */
void Ctrl_ClosedLoop_Update(void);

/** Per-axis state queries / 閫愯酱鐘舵€佹煡�?*/
bool          Ctrl_ClosedLoop_IsAxisEnabled(int axis);
void          Ctrl_ClosedLoop_SetAxisEnabled(int axis, bool en);

/** Get latest encoder angle for an axis (deg), with multi-turn update.
    鎴愬姛杩斿洖 true 骞跺啓鍏?*out_deg, 澶辫触杩斿洖 false �?*out_deg=0. */
bool          Ctrl_ClosedLoop_GetAxisAngle(int axis, float *out_deg);

/** Set zero point for an axis encoder / 璁剧疆杞寸紪鐮佸櫒闆剁偣, 鎴愬姛杩斿洖 ERR_OK */
ErrorCode_t   Ctrl_ClosedLoop_SetAxisZero(int axis);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_CLOSED_LOOP_H__ */
