/**
 * @file drv_servo.c
 * @brief 夹爪 PWM 驱动：角度换算、打开/关闭与超时停止 PWM。
 * @details 正常初始化会主动闭合夹爪；诊断入口必须跳过该初始化。
 */

#include "driver/drv_servo.h"
#include "bsp_driver_config.h"
#include "robot_math.h"

static ServoDevice_t s_gripper;

bool Drv_Servo_Bind(ServoDevice_t *grp, TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (!grp || !htim || (channel!=TIM_CHANNEL_1 && channel!=TIM_CHANNEL_2 &&
        channel!=TIM_CHANNEL_3 && channel!=TIM_CHANNEL_4)) return false;
    *grp=(ServoDevice_t){.htim=htim,.channel=channel,.state=SERVO_STATE_UNKNOWN};
    return true;
}

bool Drv_Servo_TryInit(ServoDevice_t *grp, TIM_HandleTypeDef *htim, uint32_t channel)
{
    return Drv_Servo_Bind(grp,htim,channel) && Drv_Servo_TryClose(grp);
}

void Drv_Servo_Init(ServoDevice_t *grp, TIM_HandleTypeDef *htim, uint32_t channel)
{ (void)Drv_Servo_TryInit(grp,htim,channel); }

bool Drv_Servo_TrySetAngle(ServoDevice_t *grp, float angle)
{
    if (!grp || !grp->htim || !isfinite(angle)) return false;
    if (!isfinite(GRIPPER_SERVO_MIN_US) || !isfinite(GRIPPER_SERVO_MAX_US) ||
        GRIPPER_SERVO_MIN_US<0.0f || GRIPPER_SERVO_MAX_US<=GRIPPER_SERVO_MIN_US ||
        GRIPPER_SERVO_MAX_US>20000.0f) return false;

    float safe_angle  = CLAMP(angle, 0.0f, 180.0f);
    float pulse_width = GRIPPER_SERVO_MIN_US
                      + (safe_angle / 180.0f) * (GRIPPER_SERVO_MAX_US - GRIPPER_SERVO_MIN_US);

    /* 停止失败后的通道状态不可信，不能把新目标当作故障恢复。
     * 由显式停止/安全恢复路径先确认输出关闭，再重新启动。 */
    if (grp->pwm_active && (grp->state==SERVO_STATE_UNKNOWN ||
        grp->error!=GRIPPER_ERROR_NONE)) return false;

    /* 通道已运行时只更新CCR，不再次调用HAL_TIM_PWM_Start：STM32 HAL会把
     * BUSY通道的重复启动判为HAL_ERROR。快速开→关应替换目标并从本次受理
     * 起保持600ms，而不是误报启动失败并停止PWM。停止状态仍先预载再启动。 */
    __HAL_TIM_SET_COMPARE(grp->htim, grp->channel, (uint32_t)pulse_width);
    if (!grp->pwm_active && HAL_TIM_PWM_Start(grp->htim, grp->channel)!=HAL_OK) {
        grp->state=SERVO_STATE_UNKNOWN;
        grp->last_target=GRIPPER_TARGET_NONE;
        if (grp->error==GRIPPER_ERROR_NONE) grp->error=GRIPPER_ERROR_START;
        grp->stopped_elapsed_ms=0U;
        grp->pwm_active=HAL_TIM_PWM_Stop(grp->htim,grp->channel)!=HAL_OK;
        grp->hold_start=HAL_GetTick();
        return false;
    }

    grp->cur_angle  = safe_angle;
    grp->state      = SERVO_STATE_CUSTOM_ANGLE;
    grp->hold_start = HAL_GetTick();
    grp->pwm_active = true;
    grp->last_target = GRIPPER_TARGET_CUSTOM;
    grp->stopped_elapsed_ms = 0U;
    grp->error = GRIPPER_ERROR_NONE;
    return true;
}

void Drv_Servo_SetAngle(ServoDevice_t *grp, float angle)
{
    (void)Drv_Servo_TrySetAngle(grp,angle);
}

/** 同一目标正在保持时幂等受理，不重启PWM、不延长原600ms窗口。
 * true只表示目标已发送（或正在保持），不表示传感器确认了机械到位。
 * 启动失败必须逐层返回false，不能仅凭安全许可给上位机成功回复。
 */
bool Drv_Servo_TryOpen(ServoDevice_t *grp)
{
    if (!grp || !grp->htim) return false;
    if (grp->state == SERVO_STATE_OPEN && grp->pwm_active) return true;
    if (!Drv_Servo_TrySetAngle(grp, GRIPPER_OPEN_ANGLE_DEG)) return false;
    grp->state = SERVO_STATE_OPEN;
    grp->last_target = GRIPPER_TARGET_OPEN;
    return true;
}

bool Drv_Servo_TryClose(ServoDevice_t *grp)
{
    if (!grp || !grp->htim) return false;
    if (grp->state == SERVO_STATE_CLOSE && grp->pwm_active) return true;
    if (!Drv_Servo_TrySetAngle(grp, GRIPPER_CLOSE_ANGLE_DEG)) return false;
    grp->state = SERVO_STATE_CLOSE;
    grp->last_target = GRIPPER_TARGET_CLOSE;
    return true;
}

/* 保留原void入口供既有启动代码使用；需要结果的业务应调用Try接口。 */
void Drv_Servo_Open(ServoDevice_t *grp) { (void)Drv_Servo_TryOpen(grp); }
void Drv_Servo_Close(ServoDevice_t *grp) { (void)Drv_Servo_TryClose(grp); }

bool Drv_Servo_TryStop(ServoDevice_t *grp)
{
    if (!grp || !grp->htim) return false;
    bool stopped=HAL_TIM_PWM_Stop(grp->htim,grp->channel)==HAL_OK;
    Drv_Servo_RecordStopResult(grp,stopped);
    return stopped;
}

void Drv_Servo_RecordStopResult(ServoDevice_t *grp, bool stopped)
{
    if (!grp || !grp->htim) return;
    if (grp->pwm_active) {
        uint32_t elapsed=(uint32_t)(HAL_GetTick()-grp->hold_start);
        grp->stopped_elapsed_ms=elapsed<GRIPPER_HOLD_MS?elapsed:GRIPPER_HOLD_MS;
    }
    grp->state=SERVO_STATE_UNKNOWN;
    if (stopped) {
        grp->hold_start=0U;
        grp->pwm_active=false;
    } else if (grp->error==GRIPPER_ERROR_NONE) {
        grp->error=GRIPPER_ERROR_STOP;
    }
}

void Drv_Servo_GetStatus(const ServoDevice_t *grp, GripperStatus_t *out)
{
    if (!out) return;
    *out=(GripperStatus_t){.hold_ms=GRIPPER_HOLD_MS};
    if (!grp || !grp->htim) return;
    out->bound=true;
    out->target=grp->last_target;
    out->target_angle_deg=grp->last_target==GRIPPER_TARGET_NONE?0.0f:grp->cur_angle;
    out->pwm_requested=grp->pwm_active;
    uint32_t elapsed=grp->pwm_active?(uint32_t)(HAL_GetTick()-grp->hold_start):grp->stopped_elapsed_ms;
    out->elapsed_ms=elapsed<GRIPPER_HOLD_MS?elapsed:GRIPPER_HOLD_MS;
    out->error=grp->error;
}

void Drv_Servo_Stop(ServoDevice_t *grp)
{
    (void)Drv_Servo_TryStop(grp);
}

/** 超过保持时间后停 PWM；停止 PWM 不代表夹爪已通过传感器确认到位。 */
void Drv_Servo_IdleStop(ServoDevice_t *grp)
{
    if (!grp || !grp->pwm_active) return;
    if (HAL_GetTick() - grp->hold_start >= GRIPPER_HOLD_MS) {
        (void)Drv_Servo_TryStop(grp);
    }
}

ServoDevice_t *Drv_Servo_GetHandle(void)
{
    return &s_gripper;
}
