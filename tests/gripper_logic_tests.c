/** 不接硬件；验证各限位初态、仅一次目标、租约/窗口/主循环和计时回绕。 */
#include "gripper_logic.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"grip line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static GripTest Start(uint32_t now,uint8_t limits,GripTarget target)
{GripTest t;GripTest_Init(&t);CHECK(GripTest_Start(&t,now,target,true,true,limits,true));return t;}
int main(void)
{
    for(unsigned mask=0;mask<8;mask++)for(unsigned target=1;target<=2;target++) {
        GripTest t=Start(0,(uint8_t)mask,(GripTarget)target);
        for(uint32_t now=1;now<=600;now++) {
            if(now%80U==0)CHECK(GripTest_Keep(&t,now));
            GripTest_Tick(&t,now,now,(uint8_t)mask,true);
            CHECK(t.state==(now<600?GRIP_ACTIVE:GRIP_DONE));
        }
        CHECK(t.used && !GripTest_Keep(&t,601));
        CHECK(!GripTest_Start(&t,601,GRIP_OPEN,true,true,(uint8_t)mask,true));
        CHECK(GripTest_TargetAngle(t.target)==(target==1?100.0f:72.0f));
        CHECK(GripTest_TargetPulse(t.target)==(target==1?1611U:1300U));
    }
    GripTest t;GripTest_Init(&t);
    CHECK(!GripTest_Start(NULL,0,GRIP_OPEN,true,true,0,true));
    CHECK(!GripTest_Start(&t,0,GRIP_OPEN,false,true,0,true));
    CHECK(!GripTest_Start(&t,0,GRIP_OPEN,true,false,0,true));
    CHECK(!GripTest_Start(&t,0,GRIP_OPEN,true,true,0,false));
    CHECK(!GripTest_Start(&t,0,GRIP_NONE,true,true,0,true));
    CHECK(!GripTest_Start(&t,0,(GripTarget)3,true,true,0,true));
    CHECK(!GripTest_Start(&t,0,GRIP_OPEN,true,true,8,true));CHECK(!t.used && t.state==GRIP_IDLE);
    for(unsigned mask=0;mask<8;mask++)for(unsigned bit=0;bit<3;bit++) {
        t=Start(0,(uint8_t)mask,GRIP_OPEN);GripTest_Tick(&t,1,1,(uint8_t)(mask^(1U<<bit)),true);
        CHECK(t.state==GRIP_FAILED && t.reason==GRIP_LIMIT_CHANGED);
    }
    t=Start(0,0,GRIP_OPEN);GripTest_Tick(&t,250,250,0,true);CHECK(t.reason==GRIP_HOST_LOST);
    CHECK(!GripTest_Keep(&t,251));GripTest_Stop(&t,GRIP_HOST_STOP);CHECK(t.reason==GRIP_HOST_LOST);
    t=Start(0,0,GRIP_CLOSE);CHECK(GripTest_Keep(&t,199));GripTest_Tick(&t,200,0,0,true);CHECK(t.reason==GRIP_MAIN_STALE);
    t=Start(0,0,GRIP_CLOSE);GripTest_Tick(&t,1,1,0,false);CHECK(t.reason==GRIP_INPUT_FAULT);
    t=Start(0,0,GRIP_CLOSE);GripTest_Stop(&t,GRIP_HOST_STOP);CHECK(t.state==GRIP_STOPPED);
    t=Start(0,0,GRIP_OPEN);GripTest_Stop(&t,GRIP_PWM_ERROR);CHECK(t.reason==GRIP_PWM_ERROR);
    t=Start(UINT32_MAX-300U,2,GRIP_CLOSE);uint32_t began=t.started_ms;
    for(uint32_t dt=1;dt<=600;dt++) {
        uint32_t now=began+dt;if(dt%80U==0)CHECK(GripTest_Keep(&t,now));
        GripTest_Tick(&t,now,now,2,true);
    }
    CHECK(t.state==GRIP_DONE);
    CHECK(!GripTest_Keep(NULL,0));GripTest_Init(NULL);GripTest_Stop(NULL,GRIP_HOST_STOP);
    puts("gripper_logic: 16 initial-mask/target combinations; 24 changed-mask cases; compile gate, one-shot, tick0/wrap, lease and main heartbeat passed");
    return 0;
}
