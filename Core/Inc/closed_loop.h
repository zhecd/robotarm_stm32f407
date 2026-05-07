#ifndef __CLOSED_LOOP_H__
#define __CLOSED_LOOP_H__

#include <stdbool.h>
#include <stdint.h>
#include "bsp_as5600.h"

#define CL_AXIS_COUNT   3

/* ── 单轴闭环控制器 ── */
typedef struct {
    /* PID 参数 (可在线调参) */
    float kp, ki, kd;
    float deadband_deg;       /* 死区 (度) */

    /* 运行时状态 (只读) */
    AS5600_t *encoder;
    float target_deg;
    bool enabled;
} MotorCL_t;

/* 三轴控制器数组 */
extern MotorCL_t g_axis[CL_AXIS_COUNT];

/* ── API ── */
void CL_Init(void);
void CL_SyncTarget(void);     /* 从理论步数同步目标角度 */
void CL_Update(void);         /* 50Hz 闭环更新, 由主循环调用 */

#endif
