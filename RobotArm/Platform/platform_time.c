/**
 * @file platform_time.c
 * @brief 平台时间：毫秒时基和微秒读数；业务层通过此接口替换测试时钟。
 * @details SysTick持续累计DWT周期差及余数，微秒结果按uint32_t自然回绕。
 */
#include "platform_time.h"

#include "main.h"
#include "platform_critical.h"
#include <stdbool.h>

static uint32_t s_last_cycles, s_microseconds, s_cycle_remainder, s_cycles_per_us;
static bool s_initialized;

/* 由SysTick或NowUs在短临界区调用；只做32位整数运算，保留不足1us的余数。
 * 相邻累计间隔必须小于一次CYCCNT回绕（168MHz时约25.56秒）；正常由1ms
 * SysTick保证，不依赖业务是否频繁读取NowUs。禁止运行期间随意改CPU时钟或DWT。
 */
static void AccumulateCycles(void)
{
    if (!s_initialized || s_cycles_per_us==0U) return;
    uint32_t cycles=DWT->CYCCNT;
    uint32_t elapsed=cycles-s_last_cycles;
    s_last_cycles=cycles;
    s_microseconds+=elapsed/s_cycles_per_us;
    s_cycle_remainder+=elapsed%s_cycles_per_us;
    if (s_cycle_remainder>=s_cycles_per_us) {
        ++s_microseconds;
        s_cycle_remainder-=s_cycles_per_us;
    }
}

void PlatformTime_Init(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_cycles_per_us=SystemCoreClock/1000000U;
    s_last_cycles=s_microseconds=s_cycle_remainder=0U;
    s_initialized=true;
    PlatformCritical_Exit(key);
}

void PlatformTime_OnTickFromISR(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    AccumulateCycles();
    PlatformCritical_Exit(key);
}

uint32_t PlatformTime_NowMs(void)
{
    return HAL_GetTick();
}

uint32_t PlatformTime_NowUs(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    AccumulateCycles();
    uint32_t us=s_microseconds;
    PlatformCritical_Exit(key);
    return us;
}
