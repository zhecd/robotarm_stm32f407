#include "bsp_gripper.h"

// 实例化机械臂末端夹爪
Gripper_HandleTypeDef hgripper;

// 辅助宏: 限制角度输入，保护机械结构
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/**
  * @brief  初始化夹爪对象
  */
void BSP_Gripper_Init(Gripper_HandleTypeDef *hgrp, TIM_HandleTypeDef *htim, uint32_t channel) {
    if (hgrp == NULL || htim == NULL) return;

    hgrp->htim = htim;
    hgrp->channel = channel;
    hgrp->state = GRIPPER_STATE_UNKNOWN;
    
    // 初始化时让夹爪保持张开
    BSP_Gripper_Open(hgrp);
}

/**
  * @brief  驱动舵机转到指定绝对角度
  */
void BSP_Gripper_SetAngle(Gripper_HandleTypeDef *hgrp, float angle) {
    if (hgrp == NULL) return;

    // 1. 软件限幅保护
    float safe_angle = CLAMP(angle, 0.0f, 180.0f);
    
    // 2. 线性映射: 角度(0-180) -> 脉宽(500-2500 us)
    float pulse_width = GRIPPER_SERVO_MIN_US + (safe_angle / 180.0f) * (GRIPPER_SERVO_MAX_US - GRIPPER_SERVO_MIN_US);
    
    // 3. 确保 PWM 处于启动状态
    HAL_TIM_PWM_Start(hgrp->htim, hgrp->channel);
    
    // 4. 更新占空比 (假设底层定时器计数频率为 1MHz)
    __HAL_TIM_SET_COMPARE(hgrp->htim, hgrp->channel, (uint32_t)pulse_width);
    
    // 5. 记录状态
    hgrp->cur_angle = safe_angle;
    hgrp->state = GRIPPER_STATE_CUSTOM_ANGLE;
}

/**
  * @brief  夹爪完全张开
  */
void BSP_Gripper_Open(Gripper_HandleTypeDef *hgrp) {
    if (hgrp->state != GRIPPER_STATE_OPEN) {
        BSP_Gripper_SetAngle(hgrp, GRIPPER_ANGLE_OPEN);
        hgrp->state = GRIPPER_STATE_OPEN;
    }
}

/**
  * @brief  夹爪闭合抓取
  */
void BSP_Gripper_Close(Gripper_HandleTypeDef *hgrp) {
    if (hgrp->state != GRIPPER_STATE_CLOSE) {
        BSP_Gripper_SetAngle(hgrp, GRIPPER_ANGLE_CLOSE);
        hgrp->state = GRIPPER_STATE_CLOSE;
    }
}

/**
  * @brief  切断 PWM 输出 (用于动作完成后的卸力防烧)
  */
void BSP_Gripper_Stop(Gripper_HandleTypeDef *hgrp) {
    if (hgrp == NULL) return;
    HAL_TIM_PWM_Stop(hgrp->htim, hgrp->channel);
    hgrp->state = GRIPPER_STATE_UNKNOWN;
}