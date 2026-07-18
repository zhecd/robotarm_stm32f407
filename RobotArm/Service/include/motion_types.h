#ifndef ROBOTARM_MOTION_TYPES_H
#define ROBOTARM_MOTION_TYPES_H

#include <stdint.h>

/* All public motion APIs use the same three-joint coordinate convention. */
#define MOTION_AXIS_COUNT 3U

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
    MOTION_FAULT_QUEUE_TIMEOUT
} MotionFaultReason_t;

#endif /* ROBOTARM_MOTION_TYPES_H */
