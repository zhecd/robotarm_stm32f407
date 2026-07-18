#include "app_isr_adapter.h"

#include "main.h"

#include "app/app_runtime_events.h"
#include "motion_service.h"

void App_ISR_OnTimerElapsed(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) MotionService_OnStepTickFromISR();
}

void App_ISR_OnGpioEdge(uint16_t pin)
{
    if (pin == M1_STOP_Pin || pin == M2_STOP_Pin || pin == M3_STOP_Pin) {
        MotionService_NotifyLimitSwitch(pin);
    } else if (pin == KEY_MODE_Pin) {
        AppRuntime_RequestModeSwitch();
    }
}
