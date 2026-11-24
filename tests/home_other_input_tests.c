/** GPIO回零：非当前轴首样本停步、确认后报错或恢复，故障输入快照保留。 */
#include "common/home_sequence.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void Start(HomeSequence *s,uint32_t base)
{
    HomeConfig c={20,30000,220000,{100,100,100},4,2,true};
    HomeSequence_Init(s,&c);HomeSequence_Tick(s,base,0);HomeSequence_Tick(s,base+10,0);
    assert(HomeSequence_Start(s,base+11,0));HomeSequence_Tick(s,base+31,0);assert(s->seek[0]==1);
}
int main(int argc,char **argv)
{
    assert(argc==2);const char *m=argv[1];HomeSequence s;
    uint32_t base=!strcmp(m,"wrap")?UINT32_MAX-35U:0;
    Start(&s,base);
    if(!strcmp(m,"backoff") || !strcmp(m,"rearmed") || !strcmp(m,"completed")){
        assert(HomeSequence_OnEdge(&s,0,base+32,1));
        for(unsigned t=33;t<=42;t++)HomeSequence_Tick(&s,base+t,1);
        assert(s.phase==HOME_BACKOFF);
        uint32_t t=43;
        if(strcmp(m,"backoff")){
            for(;t<=54;t++)HomeSequence_Tick(&s,base+t,0);
            assert(s.rearm_request==1);HomeSequence_AckRearm(&s,0,true);
            if(!strcmp(m,"completed")){
                for(;s.phase==HOME_BACKOFF || s.phase==HOME_RELEASE;t++)HomeSequence_Tick(&s,base+t,0);
                assert(s.phase==HOME_GAP && s.completed==1);
            }
        }
        uint8_t bad=!strcmp(m,"backoff")?2U:1U;uint32_t n=s.back[0];
        HomeSequence_Tick(&s,base+t,bad);
        assert(s.holding && !s.enable && !s.high && s.back[0]==n && s.reason==HOME_OK);
        for(unsigned i=1;i<=11;i++)HomeSequence_Tick(&s,base+t+i,0);
        assert(!s.holding && s.reason==HOME_OK && s.back[0]<=n+1U);
        if(!strcmp(m,"backoff"))assert(s.back_pause_ms==11);
    }else if(!strcmp(m,"deadline")){
        s.config.seek_timeout_ms=100;HomeSequence_Tick(&s,base+105,2);
        assert(s.holding);HomeSequence_Tick(&s,base+111,0);
        assert(s.reason==HOME_SEEK_TIMEOUT && !s.enable);
    }else{
        uint8_t bit=!strcmp(m,"glitch3")?4U:2U;
        HomeSequence_Tick(&s,base+32,bit);
        assert(s.holding && !s.enable && !s.high && s.seek[0]==1 && s.reason==HOME_OK);
        if(!strcmp(m,"held")){
            for(unsigned t=33;t<42;t++){HomeSequence_Tick(&s,base+t,bit);assert(s.reason==HOME_OK && !s.high);}
            HomeSequence_Tick(&s,base+42,bit);
            assert(s.reason==HOME_UNEXPECTED && s.failure_low==bit && s.failure_ms==base+42);
            HomeSequence_Tick(&s,base+50,0);assert(s.failure_low==bit && !s.enable);
        }else if(!strcmp(m,"unstable")){
            for(unsigned t=33;t<=132;t++)HomeSequence_Tick(&s,base+t,t%2?bit:0);
            assert(s.reason==HOME_UNSTABLE && !s.enable && s.seek[0]==1);
        }else if(!strcmp(m,"gap")){
            HomeSequence_Tick(&s,base+45,bit);assert(s.reason==HOME_OK && !s.high);
            for(unsigned t=46;t<55;t++){HomeSequence_Tick(&s,base+t,bit);assert(s.reason==HOME_OK);}
            HomeSequence_Tick(&s,base+55,bit);assert(s.reason==HOME_UNEXPECTED);
        }else{
            if(!strcmp(m,"repeat"))for(unsigned n=0;n<1000;n++)HomeSequence_Tick(&s,base+32,bit);
            unsigned release=33;
            if(!strcmp(m,"pulse9")){
                for(unsigned t=33;t<=40;t++){HomeSequence_Tick(&s,base+t,bit);assert(s.reason==HOME_OK);}
                release=41;
            }
            for(unsigned t=release;t<release+10;t++){
                HomeSequence_Tick(&s,base+t,0);assert(s.holding && !s.enable && !s.high && s.seek[0]==1);
            }
            HomeSequence_Tick(&s,base+release+10,0);assert(!s.holding && s.reason==HOME_OK);
            assert(s.seek[0]==1 && !s.high); /* 不在恢复时补发欠步。 */
        }
    }
    printf("other input %s passed\n",m);return 0;
}
