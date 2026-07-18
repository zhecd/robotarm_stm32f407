/** @file dev_gripper.h @brief Device abstraction for the robot gripper. */
#ifndef ROBOTARM_DEV_GRIPPER_H
#define ROBOTARM_DEV_GRIPPER_H

#include <stdint.h>
#include "tim.h"

void Dev_Gripper_Init(TIM_HandleTypeDef *timer, uint32_t channel);
void Dev_Gripper_Open(void);
void Dev_Gripper_Close(void);
void Dev_Gripper_Service(void);

#endif /* ROBOTARM_DEV_GRIPPER_H */
