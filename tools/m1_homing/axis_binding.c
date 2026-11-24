/** @file axis_binding.c @brief 单一板级映射，输出计划可在电脑完整枚举验证。 */
#include "axis_binding.h"
#include <stddef.h>
static const AxisBinding axes[3]={
    {1,0,0,1U,1U<<6,0U,1U<<3},
    {2,1,1,2U,1U<<7,0U,1U<<4},
    {3,2,2,4U,0U,1U<<0,1U<<5}
};
const AxisBinding *AxisBinding_Get(unsigned id)
{ return id>=1U && id<=3U?&axes[id-1U]:NULL; }

M1Inputs AxisBinding_Inputs(unsigned id,uint8_t mask,bool valid,uint32_t main_seen)
{
    const AxisBinding *a=AxisBinding_Get(id);
    if(!a)return (M1Inputs){.m1_limit=true,.other_limit=true,.feedback_valid=false,.main_seen_ms=main_seen};
    return (M1Inputs){.m1_limit=(mask&a->limit_bit)!=0U,
        .other_limit=(mask&(7U^a->limit_bit))!=0U,.feedback_valid=valid,.main_seen_ms=main_seen};
}

AxisOutputs AxisBinding_Outputs(unsigned id,const M1Test *t,bool permit,bool deadline)
{
    const AxisBinding *a=AxisBinding_Get(id);
    bool on=a && t && permit && t->enable && !deadline;
    uint32_t set_a=on && t->step?a->step_a:0U,set_b=on && t->step?a->step_b:0U;
    uint32_t reset_a=((1U<<1)|(1U<<6)|(1U<<7))&~set_a;
    uint32_t reset_b=(1U<<0)&~set_b;
    uint32_t set_d=on?0U:1U<<2,reset_d=on?1U<<2:0U;
    if(a && t) {if(t->dir_high)set_d|=a->dir_d;else reset_d|=a->dir_d;}
    return (AxisOutputs){set_a|(reset_a<<16),set_b|(reset_b<<16),set_d|(reset_d<<16)};
}
