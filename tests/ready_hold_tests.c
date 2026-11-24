/** 有界保持纯逻辑，无HAL、串口或真实保持力。 */
#include "ready_hold.h"
#include <stdio.h>
#include <string.h>
#define CHECK(c) do{if(!(c)){fprintf(stderr,"ready hold line %d\n",__LINE__);return 2;}}while(0)
int main(int argc,char **argv)
{
    CHECK(argc==2);ReadyHold h={0};const char *mode=argv[1];
    if(!strcmp(mode,"config")){
        CHECK(!ReadyHold_Arm(NULL,0,1));CHECK(!ReadyHold_Arm(&h,0,0));
        CHECK(!ReadyHold_Arm(&h,0,10001));CHECK(ReadyHold_Tick(NULL,0,true,true,false)==READY_HOLD_FAILED);
        CHECK(ReadyHold_Tick(&h,10,true,true,false)==READY_HOLD_IDLE);return 0;
    }
    uint32_t start=!strcmp(mode,"wrap")?UINT32_MAX-20U:0U;
    CHECK(ReadyHold_Arm(&h,start,10000));CHECK(!ReadyHold_Arm(&h,start,10000));
    CHECK(ReadyHold_Tick(&h,start+9999U,true,true,false)==READY_HOLD_RUNNING);
    bool homed=strcmp(mode,"unhomed")!=0,allowed=strcmp(mode,"revoked")!=0;
    bool fault=!strcmp(mode,"fault") || !strcmp(mode,"same_tick_fault");
    uint32_t elapsed=!strcmp(mode,"same_tick_fault")?10000U:9999U;
    if(!homed || !allowed || fault){
        CHECK(ReadyHold_Tick(&h,start+elapsed,homed,allowed,fault)==READY_HOLD_FAILED);
        CHECK(ReadyHold_Tick(&h,start+11000U,true,true,false)==READY_HOLD_FAILED);
    }else{
        CHECK(ReadyHold_Tick(&h,start+10000U,true,true,false)==READY_HOLD_DONE);
        CHECK(ReadyHold_Tick(&h,start+11000U,false,false,true)==READY_HOLD_DONE);
    }
    CHECK(!ReadyHold_Arm(&h,start+12000U,10000));puts("bounded ready hold passed");return 0;
}
