/**
 * @file probe_logic.c
 * @brief GPIO样本、pending观察和IRQ预算的纯逻辑，电脑测试编译此真实实现。
 */
#include "probe_logic.h"
void ProbeLogic_Init(volatile ProbeStats_t *s, uint32_t initial_low_mask)
{
    *s=(ProbeStats_t){0};s->last_low_mask=initial_low_mask&7U;
}
void ProbeLogic_Sample(volatile ProbeStats_t *s, uint32_t low_mask)
{
    low_mask&=7U;s->samples++;
    for(uint32_t a=0;a<3;a++) {
        uint32_t bit=1U<<a;
        if(low_mask&bit)s->low_samples[a]++;
        if((low_mask&bit) && !(s->last_low_mask&bit))s->falls[a]++;
        if(!(low_mask&bit) && (s->last_low_mask&bit))s->rises[a]++;
    }
    s->last_low_mask=low_mask;
}
uint32_t ProbeLogic_Pending(volatile ProbeStats_t *s,uint32_t handled,
                           uint32_t after,uint32_t now_ms,uint32_t from_irq)
{
    handled&=7U;
    if(from_irq) { s->irq_vectors++;if(!handled)s->spurious_vectors++; }
    for(uint32_t a=0;a<3;a++) {
        uint32_t bit=1U<<a;
        if(!(handled&bit))continue;
        s->pending[a]++;
        if(after&bit)s->after_ack[a]++;
        if(from_irq && !(s->masked&bit)) {
            if((uint32_t)(now_ms-s->guard_window[a])>=10U) {
                s->guard_window[a]=now_ms;s->guard_count[a]=0U;
            }
            if(++s->guard_count[a]>=256U)s->masked|=bit;
        }
    }
    return s->masked;
}
