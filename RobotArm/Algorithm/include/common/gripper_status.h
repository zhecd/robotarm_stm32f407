/** @file gripper_status.h @brief 夹爪状态快照；纯数据，不包含HAL句柄和动作许可。 */
#ifndef ROBOTARM_GRIPPER_STATUS_H
#define ROBOTARM_GRIPPER_STATUS_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    GRIPPER_TARGET_NONE=0, GRIPPER_TARGET_OPEN, GRIPPER_TARGET_CLOSE, GRIPPER_TARGET_CUSTOM
} GripperTarget_t;
typedef enum {
    GRIPPER_ERROR_NONE=0, GRIPPER_ERROR_START, GRIPPER_ERROR_STOP
} GripperError_t;
typedef struct {
    bool bound;                    /* 软件设备已绑定，不表示供电/接线已验证。 */
    GripperTarget_t target;         /* 上次成功发送的目标，停止后保留。 */
    float target_angle_deg;         /* 目标值，不是实测角度；NONE固定为0。 */
    bool pwm_requested;            /* 驱动记录：已启动或未确认停止；不是波形测量。 */
    uint32_t hold_ms;
    uint32_t elapsed_ms;            /* 保持计时，最多显示hold_ms；不表示机械到位。 */
    GripperError_t error;           /* 发送新目标成功/重新绑定前保留首个驱动错误。 */
} GripperStatus_t;
#endif
