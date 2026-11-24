/**
 * @file app_isr_adapter.h
 * @brief 中断适配：将 HAL 定时器和 GPIO 参数转换为服务事件。
 */
#ifndef ROBOTARM_APP_ISR_ADAPTER_H
#define ROBOTARM_APP_ISR_ADAPTER_H

#include "main.h"

/* CubeMX interrupt callbacks enter the application only through this adapter. */
void App_ISR_OnTimerElapsed(TIM_HandleTypeDef *htim);
void App_ISR_OnGpioEdge(uint16_t pin);
/** 共享限位IRQ入口取证；仅WAVE诊断构建记录，正常入口不采集。 */
void App_ISR_OnLimitVectorEnter(void);
/** 共享向量退出取证，保证零/一/多路实际分发均可发布完整记录。 */
void App_ISR_OnLimitVectorExit(void);
/** GPIO配置完成后、限位NVIC开放前通知；非逐轴候选为空操作。 */
void App_ISR_OnLimitsConfigured(void);
/** 毫秒固定工作量：推进回零GPIO状态机，不读I2C/UART、不打印。 */
void App_ISR_OnMillisecond(void);

#endif /* ROBOTARM_APP_ISR_ADAPTER_H */
