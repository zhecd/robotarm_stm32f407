/**
 * @file os_adapter.c
 * @brief 历史时间接口兼容层：委托 Platform；当前工程采用裸机协作式主循环。
 */
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
