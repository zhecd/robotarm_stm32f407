/** 限位解码→LED→历史→真实HOME输入的独立回归，不连接硬件。 */
#include "m1_limits.h"
#include "m1_logic.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static M1LimitSnapshot Decode(uint32_t idr)
{ return M1Limits_Decode(idr,0x0008a055U,0x01540000U); }
static M1Inputs Input(M1LimitSnapshot l,uint32_t now)
{ return (M1Inputs){.m1_limit=(l.active_mask&1U)!=0U,.other_limit=(l.active_mask&6U)!=0U,.feedback_valid=true,.main_seen_ms=now}; }
int main(void)
{
    /* 穷举整个GPIO低16位，独立按PC10/11/12规格检查，不受LED或UART位影响。 */
    for(uint32_t raw=0;raw<65536U;raw++) {
        M1LimitSnapshot l=Decode(raw|0xa5a50000U);CHECK(l.config_ok);
        uint32_t bsrr=M1Limits_LedBsrr(l.active_mask);
        CHECK((bsrr&~0x000e000eU)==0U); /* 不得写LED1/串口/限位引脚。 */
        for(unsigned axis=0;axis<3;axis++) {
            bool low=(raw&(1U<<(10U+axis)))==0U;
            CHECK(((l.active_mask>>axis)&1U)==(unsigned)low);
            CHECK(((bsrr>>(17U+axis))&1U)==(unsigned)low); /* 低有效亮灯。 */
            CHECK(((bsrr>>(1U+axis))&1U)==(unsigned)!low);
        }
    }
    for(unsigned axis=0;axis<3;axis++) {
        uint32_t shift=20U+2U*axis;
        CHECK(!M1Limits_Decode(0xffff,1U<<shift,0x01500000U).config_ok);
        CHECK(!M1Limits_Decode(0xffff,0,0x01500000U&~(3U<<shift)).config_ok);
    }
    M1LimitTrace t={0};M1Limits_Observe(&t,0);M1Limits_Observe(&t,1);M1Limits_Observe(&t,0);
    CHECK(t.samples==3 && t.low_samples[0]==1 && t.changes[0]==2 && t.low_seen_mask==1);
    M1Limits_Observe(&t,6);CHECK(t.low_seen_mask==7 && t.low_samples[1]==1 && t.low_samples[2]==1);
    t.samples=t.low_samples[0]=t.changes[0]=UINT32_MAX;
    M1Limits_Observe(&t,1);CHECK(t.samples==UINT32_MAX && t.low_samples[0]==UINT32_MAX && t.changes[0]==UINT32_MAX);
    M1Limits_Observe(NULL,0);
    for(unsigned mask=0;mask<8;mask++) {
        M1Test state;M1Test_Init(&state);CHECK(M1Test_ConfirmDirection(&state,false));
        M1Inputs in=Input(Decode(0xffffU&~(mask<<10U)),0);
        CHECK(M1Test_Start(&state,0,true,false,true,true,&in)==(mask==0));
    }
    /* 串口即便没看到1ms低电平，采样历史仍留痕；HOME不会忽略采到的低电平。 */
    M1Test state;M1Test_Init(&state);CHECK(M1Test_ConfirmDirection(&state,false));
    M1Inputs in=Input(Decode(0xffff),0);CHECK(M1Test_Start(&state,0,true,false,true,true,&in));
    in=Input(Decode(0xfbff),1);M1Test_Tick(&state,1,&in);
    CHECK(state.state==M1_QUALIFY && !state.step && state.seek_steps==0);
    in=Input(Decode(0xffff),2);M1Test_Tick(&state,2,&in);
    CHECK(state.state==M1_FAILED && state.reason==M1_LIMIT_UNSTABLE && !state.enable);
    puts("m1_limits_tests: 65536 GPIO patterns; LED isolation; saturation; decoded HOME rejection and 1ms pulse passed");
    return 0;
}
