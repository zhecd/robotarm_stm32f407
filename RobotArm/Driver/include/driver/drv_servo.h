/**
 * @file    bsp_gripper.h
 * @brief   PWM servo gripper driver (TIM2_CH2, PA1) / PWM 閼稿灚婧€婢跺湱鍩呮す鍗炲З (TIM2_CH2, PA1)
 * @ingroup bsp
 *
 * Controls a standard RC servo (500-2500 us pulse, 50 Hz) / 閹貉冨煑閺嶅洤鍣?RC 閼稿灚婧€ (500-2500us 閼村顔? 50Hz)
 */

#ifndef __BSP_GRIPPER_H__
#define __BSP_GRIPPER_H__

#include "main.h"
#include "robot_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SERVO_STATE_UNKNOWN = 0,      /* Unknown state / 閺堫亞鐓￠悩鑸碘偓?*/
    SERVO_STATE_OPEN,             /* Fully open / 鐎瑰苯鍙忓鐘茬磻 */
    SERVO_STATE_CLOSE,            /* Fully closed / 鐎瑰苯鍙忛梻顓炴値 */
    SERVO_STATE_CUSTOM_ANGLE      /* Custom angle / 閼奉亜鐣炬稊澶庮潡鎼?*/
} ServoState_t;

typedef struct {
    TIM_HandleTypeDef *htim;        /* Timer handle / 鐎规碍妞傞崳銊ュ綖閺?*/
    uint32_t           channel;     /* Timer channel / 鐎规碍妞傞崳銊┾偓姘朵壕 */
    float              cur_angle;   /* Current angle (deg) / 瑜版挸澧犵憴鎺戝 */
    ServoState_t     state;       /* Current state / 瑜版挸澧犻悩鑸碘偓?*/
    uint32_t           hold_start;  /* Last angle-change tick for auto-off */
} ServoDevice_t;

void Drv_Servo_Init(ServoDevice_t *grp, TIM_HandleTypeDef *htim, uint32_t channel);
void Drv_Servo_SetAngle(ServoDevice_t *grp, float angle);
void Drv_Servo_Open(ServoDevice_t *grp);
void Drv_Servo_Close(ServoDevice_t *grp);
void Drv_Servo_Stop(ServoDevice_t *grp);
void Drv_Servo_IdleStop(ServoDevice_t *grp);

ServoDevice_t *Drv_Servo_GetHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_GRIPPER_H__ */
