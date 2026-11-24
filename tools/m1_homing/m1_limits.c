/** @file m1_limits.c @brief PCB10_1: PC10/11/12低有效，LED2/3/4低有效。 */
#include "m1_limits.h"
#include <stddef.h>
M1LimitSnapshot M1Limits_Decode(uint32_t idr,uint32_t moder,uint32_t pupdr)
{
    return (M1LimitSnapshot){.idr=idr,.moder=moder,.pupdr=pupdr,
        .active_mask=(uint8_t)((~(idr>>10U))&7U),
        .config_ok=(moder&0x03f00000U)==0U && (pupdr&0x03f00000U)==0x01500000U};
}
uint32_t M1Limits_LedBsrr(uint8_t active_mask)
{
    uint32_t on=((uint32_t)active_mask&7U)<<1U;
    return (on<<16U)|(0x0eU&~on);
}
static void Increment(volatile uint32_t *value)
{ if(*value!=UINT32_MAX)++*value; }
void M1Limits_Observe(volatile M1LimitTrace *t,uint8_t active_mask)
{
    if(t==NULL)return;
    uint8_t mask=active_mask&7U;
    Increment(&t->samples);t->low_seen_mask|=mask;
    for(unsigned i=0;i<3U;i++) {
        uint8_t bit=(uint8_t)(1U<<i);
        if(mask&bit)Increment(&t->low_samples[i]);
        if(t->initialized && ((mask^t->last_mask)&bit))Increment(&t->changes[i]);
    }
    t->last_mask=mask;t->initialized=true;
}
