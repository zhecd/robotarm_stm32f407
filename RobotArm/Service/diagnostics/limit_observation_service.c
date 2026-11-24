/**
 * @file limit_observation_service.c
 * @brief 原始边沿和采样消抖分开统计；独立于运动及波形停止策略。
 */
#include "limit_observation_service.h"
#include "platform_critical.h"
#include "platform_time.h"
#include <stdbool.h>
#include <stddef.h>
static volatile LimitObservationStatus_t s_status;
static uint32_t s_candidate_since[3];
static uint32_t s_burst_since[3];
static uint16_t s_burst_count[3];
void LimitObservation_Init(uint8_t mask, uint32_t now)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    s_status=(LimitObservationStatus_t){.raw_mask=(uint8_t)(mask&7U),.sample_ms=now};
    for (unsigned i=0;i<3;i++) {
        s_candidate_since[i]=s_burst_since[i]=now;
        s_burst_count[i]=0U;
    }
    PlatformCritical_Exit(key);
}
bool LimitObservation_OnEdgeFromISR(uint8_t axis)
{
    if (axis>=3U) return false;
    PlatformCriticalState_t key=PlatformCritical_Enter();
    uint8_t bit=(uint8_t)(1U<<axis);
    if ((s_status.irq_latch_mask&bit)!=0U) {
        /* 共享EXTI处理器可能因其他轴进入而再次回调已屏蔽轴；保持锁存。 */
        PlatformCritical_Exit(key);
        return true;
    }
    if (s_status.edges[axis]!=UINT32_MAX) ++s_status.edges[axis];
    uint32_t now=PlatformTime_NowMs();
    if ((uint32_t)(now-s_burst_since[axis])>=LIMIT_OBSERVATION_IRQ_WINDOW_MS) {
        s_burst_since[axis]=now;
        s_burst_count[axis]=0U;
    }
    bool limit=++s_burst_count[axis]>=LIMIT_OBSERVATION_IRQ_BURST_LIMIT;
    if (limit) s_status.irq_latch_mask|=bit;
    PlatformCritical_Exit(key);
    return limit;
}
void LimitObservation_Sample(uint8_t mask, uint32_t now)
{
    mask&=7U;
    PlatformCriticalState_t key=PlatformCritical_Enter();
    for (unsigned i=0;i<3;i++) {
        uint8_t bit=(uint8_t)(1U<<i);
        if (((mask^s_status.raw_mask)&bit)!=0U) s_candidate_since[i]=now;
        if ((uint32_t)(now-s_candidate_since[i])<10U) continue;
        bool was_valid=(s_status.valid_mask&bit)!=0U;
        bool was_pressed=(s_status.stable_mask&bit)!=0U;
        bool pressed=(mask&bit)!=0U;
        if (pressed && (!was_valid || !was_pressed) && s_status.presses[i]!=UINT32_MAX)
            ++s_status.presses[i];
        s_status.valid_mask|=bit;
        if (pressed) s_status.stable_mask|=bit;
        else s_status.stable_mask&=(uint8_t)~bit;
    }
    s_status.raw_mask=mask;
    s_status.sample_ms=now;
    PlatformCritical_Exit(key);
}
void LimitObservation_GetStatus(LimitObservationStatus_t *out)
{
    if (out==NULL) return;
    PlatformCriticalState_t key=PlatformCritical_Enter();
    *out=s_status;
    PlatformCritical_Exit(key);
}
