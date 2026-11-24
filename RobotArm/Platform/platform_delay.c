/**
 * @file platform_delay.c
 * @brief 裸机延时：属于阻塞操作，仅在初始化等允许阻塞的路径使用。
 */
#include "platform_delay.h"

#include "main.h"

void PlatformDelay_Ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

void PlatformDelay_Us(uint32_t delay_us)
{
    uint32_t cycles = delay_us * (SystemCoreClock / 8000000U);
    for (volatile uint32_t i = 0U; i < cycles; i++) {
        __NOP();
    }
}
