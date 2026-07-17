/** @file dev_limit_switch.c @brief Active-low limit-switch adapter. */
#include "device/dev_limit_switch.h"
#include "main.h"

bool Dev_LimitSwitch_IsTriggered(DevJointId_t joint)
{
    switch (joint) {
    case DEV_JOINT_M1:
        return HAL_GPIO_ReadPin(M1_STOP_GPIO_Port, M1_STOP_Pin) == GPIO_PIN_RESET;
    case DEV_JOINT_M2:
        return HAL_GPIO_ReadPin(M2_STOP_GPIO_Port, M2_STOP_Pin) == GPIO_PIN_RESET;
    case DEV_JOINT_M3:
        return HAL_GPIO_ReadPin(M3_STOP_GPIO_Port, M3_STOP_Pin) == GPIO_PIN_RESET;
    default:
        return false;
    }
}

bool Dev_LimitSwitch_IsPinTriggered(uint16_t gpio_pin)
{
    switch (gpio_pin) {
    case M1_STOP_Pin: return Dev_LimitSwitch_IsTriggered(DEV_JOINT_M1);
    case M2_STOP_Pin: return Dev_LimitSwitch_IsTriggered(DEV_JOINT_M2);
    case M3_STOP_Pin: return Dev_LimitSwitch_IsTriggered(DEV_JOINT_M3);
    default: return false;
    }
}
