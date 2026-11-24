/**
 * @file limit_irq_guard.c
 * @brief 每轴有界回调预算：时基被冻结时仍可达到阈值并锁存，释放不自动恢复。
 */
#include "limit_irq_guard.h"
#include "platform_critical.h"
#include "platform_time.h"
static uint8_t s_latch_mask;
static uint16_t s_count[3];
static uint32_t s_window_ms[3];
bool LimitIrqGuard_OnEdgeFromISR(uint8_t axis)
{
    if (axis>=3U) return false;
    PlatformCriticalState_t key=PlatformCritical_Enter();
    uint8_t bit=(uint8_t)(1U<<axis);
    if ((s_latch_mask&bit)==0U) {
        uint32_t now=PlatformTime_NowMs();
        if ((uint32_t)(now-s_window_ms[axis])>=LIMIT_IRQ_GUARD_WINDOW_MS) {
            s_window_ms[axis]=now;s_count[axis]=0U;
        }
        if (++s_count[axis]>=LIMIT_IRQ_GUARD_BURST_LIMIT) s_latch_mask|=bit;
    }
    bool latched=(s_latch_mask&bit)!=0U;
    PlatformCritical_Exit(key);
    return latched;
}
uint8_t LimitIrqGuard_GetLatchMask(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    uint8_t mask=s_latch_mask;
    PlatformCritical_Exit(key);return mask;
}
