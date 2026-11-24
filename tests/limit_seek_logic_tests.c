#include "seek_logic.h"
#include <assert.h>
#include <stdio.h>
static void complete_axis(Seek *s, uint32_t *now)
{
    unsigned a=s->axis;
    for(unsigned i=0;i<81;i++)Seek_Tick(s,++*now,s->completed);
    assert(s->steps[a]==4U);
    uint8_t mask=(uint8_t)(s->completed|(1U<<a));
    Seek_Tick(s,++*now,mask);
    assert(!s->enable && !s->high && s->state==SEEK_QUALIFY);
    for(unsigned i=0;i<10;i++)Seek_Tick(s,++*now,mask);
    assert(s->completed==mask);
    if(a<2U)for(unsigned i=0;i<100;i++)Seek_Tick(s,++*now,mask);
}
int main(void)
{
    Seek s={0};uint32_t t=0;
    assert(!Seek_Start(NULL,0,0));assert(!Seek_Start(&s,0,1));assert(!s.used);
    assert(Seek_Start(&s,t,0));assert(!Seek_Start(&s,t,0));
    complete_axis(&s,&t);assert(s.axis==1);complete_axis(&s,&t);
    assert(s.axis==2);complete_axis(&s,&t);
    assert(s.state==SEEK_DONE && s.completed==7 && !s.enable && !s.high);
    Seek_Tick(&s,t+100000,0);assert(s.state==SEEK_DONE);
    assert(!Seek_Start(&s,t,0));
    s=(Seek){0};assert(Seek_Start(&s,0,0));
    unsigned pulses=0;
    for(t=1;t<5000;t++) {Seek_Tick(&s,t,0);pulses+=s.high;}
    assert(s.state==SEEK_FAILED && s.reason==SEEK_BUDGET && s.steps[0]==200 && pulses==200);
    assert(!s.enable && !s.high && s.steps[1]==0 && s.steps[2]==0);
    s=(Seek){0};assert(Seek_Start(&s,0,0));
    Seek_Tick(&s,20,1);assert(s.steps[0]==0 && !s.high && !s.enable);
    Seek_Tick(&s,21,0);assert(s.state==SEEK_QUALIFY && !s.enable);
    Seek_Tick(&s,120,0);assert(s.reason==SEEK_UNSTABLE && s.state==SEEK_FAILED);
    s=(Seek){0};assert(Seek_Start(&s,0,0));Seek_Tick(&s,1,4);
    assert(s.reason==SEEK_UNEXPECTED && !s.enable);
    s=(Seek){0};assert(Seek_Start(&s,0,0));Seek_Tick(&s,4500,0);
    assert(s.reason==SEEK_TIME && s.steps[0]==0);
    s=(Seek){0};t=0xfffffff0U;assert(Seek_Start(&s,t,0));Seek_Tick(&s,t+20U,0);
    assert(s.high && s.steps[0]==1);Seek_Tick(&s,t+21U,0);assert(!s.high);
    /* 调度延误不补发脉冲，避免一次追赶造成突发。 */
    Seek_Tick(&s,t+200U,0);assert(s.steps[0]==2);
    Seek_Stop(&s,SEEK_IRQ);assert(s.state==SEEK_FAILED && !s.enable && !s.high);
    Seek_Stop(&s,SEEK_ABORT);assert(s.reason==SEEK_IRQ);
    /* 第200步后采到限位仍可正常确认，不误报预算、不发201步。 */
    s=(Seek){0};assert(Seek_Start(&s,0,0));
    for(t=1;t<=4000;t++)Seek_Tick(&s,t,0);
    Seek_Tick(&s,4001,1);Seek_Tick(&s,4011,1);
    assert(s.state==SEEK_GAP && s.steps[0]==200 && s.completed==1);
    puts("limit_seek: sequence, cap, contact, unstable, unexpected, deadline, wrap, pulse, no catch-up, terminal, final-step contact passed");
    return 0;
}
