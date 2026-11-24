#include "seek_logic.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    Seek s={0};uint32_t t=0;
    /* M1已压住时只确认，不发步；仍不发布全局零位。 */
    assert(Seek_Start(&s,t,1));assert(!s.enable && s.state==SEEK_QUALIFY);
    for(t=1;t<=110;t++)Seek_Tick(&s,t,1);
    assert(s.axis==1 && s.completed==1 && s.steps[0]==0);
    for(t=111;t<=10110;t++)Seek_Tick(&s,t,1);
    assert(s.steps[1]==500 && s.state==SEEK_MOVING); /* 已超过旧5°停止点。 */
    Seek_Tick(&s,10111,3);assert(!s.enable && !s.high);
    for(t=10112;t<=10221;t++)Seek_Tick(&s,t,3);
    assert(s.axis==2 && s.completed==3);
    for(t=10222;t<=16221;t++)Seek_Tick(&s,t,3);
    assert(s.steps[2]==300);Seek_Tick(&s,16222,7);Seek_Tick(&s,16232,7);
    assert(s.state==SEEK_DONE && s.completed==7 && !s.enable && !s.high);
    assert(s.steps[0]==0 && s.steps[1]==500 && s.steps[2]==300);
    s=(Seek){0};assert(Seek_Start(&s,0,0));
    for(t=1;t<=30000;t++)Seek_Tick(&s,t,0);
    assert(s.state==SEEK_FAILED && s.reason==SEEK_TIME && !s.enable && !s.high);
    assert(s.steps[0]==1499 && s.steps[1]==0 && s.steps[2]==0);
    s=(Seek){0};assert(Seek_Start(&s,0,7));
    for(t=1;t<=250;t++)Seek_Tick(&s,t,7);
    assert(s.state==SEEK_DONE && s.completed==7);
    assert(s.steps[0]==0 && s.steps[1]==0 && s.steps[2]==0);
    s=(Seek){0};assert(Seek_Start(&s,0,1));Seek_Tick(&s,1,0);
    assert(s.state==SEEK_QUALIFY && !s.enable);
    Seek_Tick(&s,100,0);
    assert(s.state==SEEK_FAILED && s.reason==SEEK_UNSTABLE);
    s=(Seek){0};assert(Seek_Start(&s,0,0));Seek_Tick(&s,1,2);
    assert(s.state==SEEK_FAILED && s.reason==SEEK_UNEXPECTED);
    s=(Seek){0};assert(!Seek_Start(&s,0,8));
    s=(Seek){0};assert(Seek_Start(&s,0,0));s.steps[0]=1500;Seek_Tick(&s,20,0);
    assert(s.state==SEEK_FAILED && s.reason==SEEK_BUDGET && s.steps[0]==1500);
    puts("extended seek: >5deg contact, pressed-start, zero-step skip, timeout, invalid input, cap passed");
    return 0;
}
