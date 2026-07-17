#ifndef ROBOTARM_APP_ISR_H
#define ROBOTARM_APP_ISR_H

#include "main.h"

void App_ISR_OnTimerElapsed(TIM_HandleTypeDef *htim);
void App_ISR_OnGpioEdge(uint16_t pin);

#endif
