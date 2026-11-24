/** 当前轴接触短脉冲：首低停步，稳定高10ms取消，原截止不延长。 */
#include "common/home_sequence.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
int main(int argc,char **argv)
{
    assert(argc==2);const char *m=argv[1];uint32_t base=!strcmp(m,"wrap")?UINT32_MAX-20U:0;
    HomeConfig c={20,30000,220000,{100,100,100},4,2,true};HomeSequence s;
    HomeSequence_Init(&s,&c);HomeSequence_Tick(&s,base,0);HomeSequence_Tick(&s,base+10,0);
    assert(HomeSequence_Start(&s,base+11,0));HomeSequence_Tick(&s,base+12,1);
    assert(s.phase==HOME_CONTACT && !s.enable && !s.high && !s.seek[0]);
    if(!strcmp(m,"held")){
        for(unsigned t=13;t<22;t++){HomeSequence_Tick(&s,base+t,1);assert(s.phase==HOME_CONTACT && !s.high);}
        HomeSequence_Tick(&s,base+22,1);assert(s.phase==HOME_BACKOFF && s.contacts==1 && !s.contact_cancel_count);
    }else if(!strcmp(m,"bounce")){
        for(unsigned t=13;t<=112;t++)HomeSequence_Tick(&s,base+t,(t&1U)?0:1);
        assert(s.reason==HOME_UNSTABLE && !s.enable && !s.contact_cancel_count);
    }else if(!strcmp(m,"deadline") || !strcmp(m,"total")){
        if(!strcmp(m,"deadline"))s.config.seek_timeout_ms=100;else s.config.total_timeout_ms=100;
        for(unsigned t=13;t<=111;t++)HomeSequence_Tick(&s,base+t,t%30U==0U?1U:0U);
        assert(s.reason==(!strcmp(m,"deadline")?HOME_SEEK_TIMEOUT:HOME_TOTAL_TIMEOUT) && !s.enable);
        assert(s.began==base+11 && s.axis_began==base+11); /* 每次取消均不重置截止。 */
    }else if(!strcmp(m,"gap")){
        HomeSequence_Tick(&s,base+13,0);HomeSequence_Tick(&s,base+30,0);
        assert(s.phase==HOME_CONTACT && !s.enable);
        for(unsigned t=31;t<40;t++){HomeSequence_Tick(&s,base+t,0);assert(s.phase==HOME_CONTACT);}
        HomeSequence_Tick(&s,base+40,0);assert(s.phase==HOME_SEEK && s.contact_cancel_count==1);
    }else{
        unsigned high=!strcmp(m,"low9")?21U:13U;
        for(unsigned t=13;t<high;t++)HomeSequence_Tick(&s,base+t,1);
        for(unsigned t=high;t<high+10;t++){
            HomeSequence_Tick(&s,base+t,0);assert(s.phase==HOME_CONTACT && !s.enable && !s.high);
        }
        for(unsigned n=0;n<100;n++)HomeSequence_Tick(&s,base+high+9,0);
        assert(s.phase==HOME_CONTACT); /* 重复同一毫秒不会加速确认。 */
        HomeSequence_Tick(&s,base+high+10,0);
        assert(s.phase==HOME_SEEK && s.enable && !s.high && !s.seek[0] && s.contact_cancel_count==1);
        HomeSequence_Tick(&s,base+high+29,0);assert(!s.seek[0] && !s.high);
        HomeSequence_Tick(&s,base+high+30,0);assert(s.seek[0]==1 && s.high);
    }
    printf("contact cancel %s passed\n",m);return 0;
}
