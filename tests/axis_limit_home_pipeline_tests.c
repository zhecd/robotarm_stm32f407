/**
 * @file axis_limit_home_pipeline_tests.c
 * @brief 三轴实际纯模块串联：GPIO快照→限位/LED/历史→选轴→HOME→GPIO输出计划。
 * 不连接串口或HAL；GPIO输入和1ms时钟由测试注入，不证明实物已接触开关。
 */
#include "axis_binding.h"
#include "m1_limits.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr,"pipeline line %d: %s\n",__LINE__,#x); exit(1); } } while (0)

typedef struct { M1Test state; M1LimitTrace trace; unsigned axis; } Fixture;

/* 物理规范独立写明PC10/11/12，不从被测描述表反推测试输入。 */
static uint32_t Raw(unsigned pressed_axis)
{ return pressed_axis ? 0xffffU & ~(1U << (pressed_axis+9U)) : 0xffffU; }
static M1LimitSnapshot Decode(uint32_t raw)
{ return M1Limits_Decode(raw,0x0008a055U,0x01540000U); }
static uint32_t Apply(uint32_t before,uint32_t bsrr)
{
    CHECK(((bsrr&0xffffU)&(bsrr>>16U))==0U);
    return (before&~(bsrr>>16U))|(bsrr&0xffffU);
}
static Fixture Start(unsigned axis,bool direction)
{
    Fixture f={.axis=axis}; M1Test_Init(&f.state);
    CHECK(M1Test_ConfirmDirection(&f.state,direction));
    M1Inputs in=AxisBinding_Inputs(axis,Decode(Raw(0)).active_mask,true,0);
    CHECK(M1Test_Start(&f.state,0,true,false,true,true,&in));
    return f;
}
static void Tick(Fixture *f,uint32_t now,uint32_t raw)
{
    M1LimitSnapshot l=Decode(raw);CHECK(l.config_ok);
    M1Limits_Observe(&f->trace,l.active_mask);
    uint32_t leds=Apply(0x000fU,M1Limits_LedBsrr(l.active_mask));
    for(unsigned id=1;id<=3;id++)
        CHECK(((leds>>id)&1U)==((raw>>(id+9U))&1U));
    CHECK((leds&1U)==1U); /* 限位LED写计划不能改LED1。 */
    M1Inputs in=AxisBinding_Inputs(f->axis,l.active_mask,true,now);
    if(M1Test_Active(&f->state) && now%40U==0U) CHECK(M1Test_KeepAlive(&f->state,now));
    M1Test_Tick(&f->state,now,&in);
    AxisOutputs o=AxisBinding_Outputs(f->axis,&f->state,true,false);
    uint32_t a=Apply(0xffffU,o.a_bsrr),b=Apply(0xffffU,o.b_bsrr),d=Apply(0xffffU,o.d_bsrr);
    uint32_t selected_a=f->axis==1?1U<<6:f->axis==2?1U<<7:0U;
    uint32_t selected_b=f->axis==3?1U:0U;
    CHECK((a&0xc2U)==(f->state.enable && f->state.step?selected_a:0U));
    CHECK((b&1U)==(f->state.enable && f->state.step?selected_b:0U));
    CHECK(((d&4U)==0U)==f->state.enable);
    CHECK(((d>>(f->axis+2U))&1U)==(unsigned)f->state.dir_high);
}
static void Success(unsigned axis,bool direction,bool last_step)
{
    Fixture f=Start(axis,direction);bool hit=false,saw_qualify=false,saw_backoff=false;
    uint32_t count_at_contact=0;
    for(uint32_t now=1;now<5300U && M1Test_Active(&f.state);now++) {
        if(!hit && (last_step?f.state.seek_steps==200U:now==45U)) {
            hit=true;count_at_contact=f.state.seek_steps;
        }
        bool pressed=hit && !(f.state.state==M1_BACKOFF && f.state.backoff_steps>=5U);
        Tick(&f,now,Raw(pressed?axis:0));
        if(f.state.state==M1_QUALIFY) {
            saw_qualify=true;CHECK(!f.state.step && f.state.seek_steps==count_at_contact);
            CHECK(f.state.dir_high==direction);
        }
        if(f.state.state==M1_BACKOFF) {
            saw_backoff=true;CHECK(f.state.dir_high!=direction);
            CHECK(f.state.seek_steps==count_at_contact);
        }
    }
    CHECK(hit && saw_qualify && saw_backoff);
    CHECK(f.state.state==M1_DONE && f.state.reason==M1_OK);
    CHECK(f.state.backoff_steps==40U && !f.state.enable && !f.state.step && f.state.used);
    CHECK(f.trace.low_samples[axis-1U]>=10U && f.trace.changes[axis-1U]==2U);
    for(unsigned id=1;id<=3;id++) if(id!=axis) CHECK(f.trace.low_samples[id-1U]==0U);
    M1Inputs open=AxisBinding_Inputs(axis,0,true,5300);
    CHECK(!M1Test_Start(&f.state,5300,true,false,true,true,&open)); /* 不自动重放。 */
}
static void RejectAndBound(unsigned axis,bool direction)
{
    Fixture f=Start(axis,direction);
    Tick(&f,1,Raw(axis));CHECK(f.state.state==M1_QUALIFY && !f.state.step);
    Tick(&f,2,Raw(0));CHECK(f.state.state==M1_FAILED && f.state.reason==M1_LIMIT_UNSTABLE);
    CHECK(!f.state.enable && f.trace.low_samples[axis-1U]==1U);

    f=Start(axis,direction);Tick(&f,1,Raw(axis%3U+1U));
    CHECK(f.state.state==M1_FAILED && f.state.reason==M1_OTHER_LIMIT && !f.state.step);

    f=Start(axis,direction);
    for(uint32_t now=1;now<4500U && M1Test_Active(&f.state);now++) Tick(&f,now,Raw(0));
    CHECK(f.state.state==M1_FAILED && f.state.reason==M1_TRAVEL_BOUND);
    CHECK(f.state.seek_steps==200U && f.state.backoff_steps==0U && !f.state.enable && !f.state.step);
    CHECK(f.trace.low_seen_mask==0U);

    M1Test state;M1Test_Init(&state);CHECK(M1Test_ConfirmDirection(&state,direction));
    M1Inputs pressed=AxisBinding_Inputs(axis,Decode(Raw(axis)).active_mask,true,0);
    CHECK(!M1Test_Start(&state,0,true,false,true,true,&pressed));
}
int main(int argc,char **argv)
{
    CHECK(argc==2);unsigned axis=(unsigned)strtoul(argv[1],NULL,10);CHECK(axis>=1U && axis<=3U);
    for(unsigned direction=0;direction<2;direction++) {
        Success(axis,direction!=0U,false);
        Success(axis,direction!=0U,true);
        RejectAndBound(axis,direction!=0U);
    }
    printf("axis%u: raw GPIO through HOME/LED/history/output; both directions, last-step contact, bounce, other limit, no-hit bound, replay passed\n",axis);
    return 0;
}
