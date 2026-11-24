/**
 * @file motion_types.h
 * @brief 运动公共数据：delta 为有符号电机微步数，total_ticks 为定时器节拍数。
 */
#ifndef ROBOTARM_MOTION_TYPES_H
#define ROBOTARM_MOTION_TYPES_H

#include <stdint.h>

/* All public motion APIs use the same three-joint coordinate convention. */
#define MOTION_AXIS_COUNT 3U
/* 逻辑轴位图，与 GPIO 引脚编号无关：bit0=M1、bit1=M2、bit2=M3。 */
#define MOTION_LIMIT_M1 (1U << 0)
#define MOTION_LIMIT_M2 (1U << 1)
#define MOTION_LIMIT_M3 (1U << 2)

/* One timer-driven, joint-space motion segment. */
typedef struct {
    int32_t  delta_m1;
    int32_t  delta_m2;
    int32_t  delta_m3;
    uint32_t total_ticks;
} MotionFrame_t;

typedef enum {
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_LIMIT_SWITCH,
    MOTION_FAULT_ENCODER,
    MOTION_FAULT_SOFT_LIMIT,
    MOTION_FAULT_QUEUE_TIMEOUT,
    MOTION_FAULT_CONTROL_DIVERGENCE,
    MOTION_FAULT_HOST_STOP,
    MOTION_FAULT_LIMIT_IRQ_OVERLOAD,
    MOTION_FAULT_INPUT_LOST,
    /* 追加编号，保持现有串口MOTION_FAULT编号兼容。 */
    MOTION_FAULT_CORRECTION_RANGE,
    MOTION_FAULT_CORRECTION_TIMEOUT,
    MOTION_FAULT_CORRECTION_STALLED
} MotionFaultReason_t;

#endif /* ROBOTARM_MOTION_TYPES_H */
