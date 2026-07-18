#ifndef ROBOTARM_APP_ISR_ADAPTER_H
#define ROBOTARM_APP_ISR_ADAPTER_H

#include "main.h"

/* CubeMX interrupt callbacks enter the application only through this adapter. */
void App_ISR_OnTimerElapsed(TIM_HandleTypeDef *htim);
void App_ISR_OnGpioEdge(uint16_t pin);

#endif /* ROBOTARM_APP_ISR_ADAPTER_H */
