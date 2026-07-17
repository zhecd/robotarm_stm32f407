/** @file os_adapter.c @brief Bare-metal implementation of OS abstractions. */
#include "os/os_adapter.h"
#include "main.h"

uint32_t Os_GetTickMs(void)
{
    return HAL_GetTick();
}

void Os_DelayMs(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

void Os_DelayUs(uint32_t delay_us)
{
    uint32_t cycles = delay_us * (SystemCoreClock / 8000000U);
    for (volatile uint32_t i = 0U; i < cycles; i++) { __NOP(); }
}
