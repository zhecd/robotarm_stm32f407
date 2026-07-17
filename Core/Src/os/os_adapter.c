/** @file os_adapter.c @brief Bare-metal implementation of OS abstractions. */
#include "os/os_adapter.h"
#include "platform_delay.h"
#include "platform_time.h"

uint32_t Os_GetTickMs(void)
{
    return PlatformTime_NowMs();
}

void Os_DelayMs(uint32_t delay_ms)
{
    PlatformDelay_Ms(delay_ms);
}

void Os_DelayUs(uint32_t delay_us)
{
    PlatformDelay_Us(delay_us);
}
