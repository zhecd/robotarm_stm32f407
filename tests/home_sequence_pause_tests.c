/** 复现123长退回+38次短暂停顿；检查有限暂停预算，不放宽总截止或脉冲间隔。 */
#include "common/home_sequence.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static HomeSequence Prepare(uint32_t base)
{
    HomeConfig c={20,30000,220000,{4000,1240,200},4,0,false};HomeSequence s;
    HomeSequence_Init(&s,&c);HomeSequence_Tick(&s,base,2);HomeSequence_Tick(&s,base+10,2);
    assert(HomeSequence_Start(&s,base+11,2));
    assert(HomeSequence_OnEdge(&s,0,base+12,3));HomeSequence_Tick(&s,base+22,3);
    assert(s.phase==HOME_BACKOFF);return s;
}
int main(int argc,char **argv)
{
    assert(argc==2);const char *mode=argv[1];
    uint32_t base=!strcmp(mode,"wrap")?UINT32_MAX-1000U:0U;
    HomeSequence s=Prepare(base);uint32_t began=s.phase_began,last_pulse=began,old_steps=0;
    if(!strcmp(mode,"total_deadline")){
        s.config.total_timeout_ms=1000;s.holding=true;s.hold_began=base+995;
        HomeSequence_Tick(&s,base+1011,2);assert(s.reason==HOME_TOTAL_TIMEOUT);
    }else if(!strcmp(mode,"seek_deadline")){
        HomeConfig c={20,100,1000,{40,40,40},4,0,false};HomeSequence_Init(&s,&c);
        HomeSequence_Tick(&s,0,2);HomeSequence_Tick(&s,10,2);assert(HomeSequence_Start(&s,11,2));
        for(uint32_t t=12;t<=111;t++)HomeSequence_Tick(&s,t,t%30==0?0:2);
        assert(s.reason==HOME_SEEK_TIMEOUT);
    }else{
        uint32_t period=!strcmp(mode,"pause_cap")?100U:!strcmp(mode,"many_pauses")?500U:2000U;
        for(uint32_t age=1;age<90000 && s.phase!=HOME_FAILED && s.phase!=HOME_GAP;age++){
            uint8_t low=2;
            bool pulse=age>=500U && (age-500U)%period==0U;
            if((!strcmp(mode,"recorded38") || !strcmp(mode,"wrap")) && age>74500U)pulse=false;
            if(pulse)low=0;
            if(s.back[0]<2U)low|=1U;
            HomeSequence_Tick(&s,began+age,low);
            if(s.rearm_request&1U)HomeSequence_AckRearm(&s,0,true);
            if(s.back[0]!=old_steps){
                assert(s.back[0]==old_steps+1U && s.high);
                assert((uint32_t)(began+age-last_pulse)>=20U);
                last_pulse=began+age;old_steps=s.back[0];
            }
        }
        if(!strcmp(mode,"pause_cap"))assert(s.phase==HOME_FAILED && s.reason==HOME_UNSTABLE && !s.enable);
        else{
            assert(s.phase==HOME_GAP && s.reason==HOME_OK && s.back[0]==4000);
            assert(s.back_pause_ms<=HOME_BACKOFF_PAUSE_MAX_MS);
            if(!strcmp(mode,"recorded38") || !strcmp(mode,"wrap"))assert(s.hold_count==38);
        }
    }
    printf("home pause %s: reason=%u steps=%lu holds=%lu pause_ms=%lu\n",mode,s.reason,
           (unsigned long)s.back[0],(unsigned long)s.hold_count,(unsigned long)s.back_pause_ms);
    return 0;
}
