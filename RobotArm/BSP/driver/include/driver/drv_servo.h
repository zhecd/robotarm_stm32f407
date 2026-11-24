/**
 * @file drv_servo.h
 * @brief 夹爪 PWM 驱动：角度换算、打开/关闭与超时停止 PWM。
 * @details 正常初始化会主动闭合夹爪；诊断入口必须跳过该初始化。
 */

#ifndef __BSP_GRIPPER_H__
#define __BSP_GRIPPER_H__

#include "main.h"
#include "robot_config.h"
#include "gripper_status.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SERVO_STATE_UNKNOWN = 0,      /* 未发送目标，或PWM已停止/发生错误。 */
    SERVO_STATE_OPEN,             /* 已发出打开目标，未测量实际到位。 */
    SERVO_STATE_CLOSE,            /* 已发出关闭目标，不表示已检测到物理到位。 */
    SERVO_STATE_CUSTOM_ANGLE      /* 自定义目标角度。 */
} ServoState_t;

typedef struct {
    TIM_HandleTypeDef *htim;      /* 底层定时器句柄，仅驱动及板级绑定使用。 */
    uint32_t           channel;     /* 定时器通道。 */
    float              cur_angle;   /* 上次成功发送的目标角度，非实际位置。 */
    ServoState_t       state;     /* 驱动记录的目标状态。 */
    uint32_t           hold_start;  /* 启动毫秒，可合法为0；用差值处理回绕。 */
    bool               pwm_active;  /* 已启动或停止失败；不代表机械到位。 */
    GripperTarget_t    last_target; /* 停PWM后仍保存目标，区别于瞬时state。 */
    uint32_t          stopped_elapsed_ms;
    GripperError_t    error;
} ServoDevice_t;

/** 只绑定并清除软件状态，不启动PWM、不自动闭合。调用前须停止旧通道。 */
bool Drv_Servo_Bind(ServoDevice_t *grp, TIM_HandleTypeDef *htim, uint32_t channel);
/** 正常启动：绑定后发出原闭合目标，返回实际启动结果。调用前须停旧通道。 */
bool Drv_Servo_TryInit(ServoDevice_t *grp, TIM_HandleTypeDef *htim, uint32_t channel);
/** 正常启动兼容入口：绑定后仍执行原有闭合；诊断准备应使用Bind。 */
void Drv_Servo_Init(ServoDevice_t *grp, TIM_HandleTypeDef *htim, uint32_t channel);
/** 主循环调用；由上层短临界区与安全停止互斥。拒绝非有限角度/无效句柄。
 * 停止态先预载再启动；正常运行态只更新CCR并重计600ms，不重复Start。
 * 停止失败的未知活动通道拒绝新目标，须先明确停止；受理不表示机械到位。 */
bool Drv_Servo_TrySetAngle(ServoDevice_t *grp, float angle);
void Drv_Servo_SetAngle(ServoDevice_t *grp, float angle);
/** true=目标发送成功或同目标正在保持；false=参数/PWM启动失败。不是物理到位。 */
bool Drv_Servo_TryOpen(ServoDevice_t *grp);
bool Drv_Servo_TryClose(ServoDevice_t *grp);
void Drv_Servo_Open(ServoDevice_t *grp);
void Drv_Servo_Close(ServoDevice_t *grp);
/** HAL停止失败时保留pwm_active，以便调用方识别并再次尝试停止。 */
bool Drv_Servo_TryStop(ServoDevice_t *grp);
/** 板级安全停止完成HAL调用后同步软件记录；可在ISR使用，不操作硬件、不打印。 */
void Drv_Servo_RecordStopResult(ServoDevice_t *grp, bool stopped);
/** 只读快照；调用方用短临界区与ISR停止记录互斥，不启动/停止PWM。 */
void Drv_Servo_GetStatus(const ServoDevice_t *grp, GripperStatus_t *out);
void Drv_Servo_Stop(ServoDevice_t *grp);
/** 超过保持时间后停 PWM；停止 PWM 不代表夹爪已通过传感器确认到位。 */
void Drv_Servo_IdleStop(ServoDevice_t *grp);

ServoDevice_t *Drv_Servo_GetHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_GRIPPER_H__ */
