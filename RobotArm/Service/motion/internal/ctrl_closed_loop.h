/**
 * @file ctrl_closed_loop.h
 * @brief 闭环内部实现：编码器采样、误差补偿和补偿发散保护。
 * @details 补偿使用电机轴角度，恢复原有目标，不修改规划器的理论位置。
 */

#ifndef __CTRL_CLOSED_LOOP_H__
#define __CTRL_CLOSED_LOOP_H__

#include <stdbool.h>
#include <stdint.h>
#include "common/error_code.h"
#include "closed_loop_status.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CL_AXIS_COUNT  3     /* 受控轴数。 */

void Ctrl_ClosedLoop_Init(void);

/* 主循环将理论微步同步为电机轴目标；重置滤波与PID，不清诊断累计数。 */
void Ctrl_ClosedLoop_SyncTarget(void);

/* 主循环每20ms推进一次；运动/稳定等待期间只采样，不新增补偿。 */
void Ctrl_ClosedLoop_Update(void);
bool Ctrl_ClosedLoop_IsRecoveryActive(void);
/* 三轴均已用新鲜反馈确认进入到位死区，稳定等待和补偿均已结束。 */
bool Ctrl_ClosedLoop_HasSettled(void);
/* 复制已有记录，无硬件访问；只能由主循环调用，NULL无操作。 */
void Ctrl_ClosedLoop_GetStatus(ClosedLoopStatus_t *out);

/* 逐轴闭环配置；axis为0/1/2，不是板端接口编号。 */
bool          Ctrl_ClosedLoop_IsAxisEnabled(int axis);
void          Ctrl_ClosedLoop_SetAxisEnabled(int axis, bool en);

/* 读取电机轴角度；失败输出 0，调用者必须检查返回值。 */
bool          Ctrl_ClosedLoop_GetAxisAngle(int axis, float *out_deg);

/* 设置轴编码器零点，成功返回 ERR_OK。 */
ErrorCode_t   Ctrl_ClosedLoop_SetAxisZero(int axis);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_CLOSED_LOOP_H__ */
