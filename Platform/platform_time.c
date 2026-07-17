#include "platform_time.h"

#include "main.h"

void PlatformTime_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t PlatformTime_NowMs(void)
{
    return HAL_GetTick();
}

uint32_t PlatformTime_NowUs(void)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    return cycles_per_us == 0U ? 0U : DWT->CYCCNT / cycles_per_us;
}
