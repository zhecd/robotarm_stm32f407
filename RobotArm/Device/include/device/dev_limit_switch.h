/** @file dev_limit_switch.h @brief Joint limit-switch abstraction. */
#ifndef ROBOTARM_DEV_LIMIT_SWITCH_H
#define ROBOTARM_DEV_LIMIT_SWITCH_H

#include <stdbool.h>
#include <stdint.h>
#include "device/dev_joint.h"

bool Dev_LimitSwitch_IsTriggered(DevJointId_t joint);
bool Dev_LimitSwitch_IsPinTriggered(uint16_t gpio_pin);

#endif /* ROBOTARM_DEV_LIMIT_SWITCH_H */
