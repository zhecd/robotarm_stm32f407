/** 1000微步/秒回零纯算法：时钟、限位由用例提供，不操作实机。 */
#include "common/home_sequence.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* 非交互失败：Windows下标准assert可能弹窗，离线负对照须立即返回非零。 */
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"home1000 line %d: %s\n",__LINE__,#x);exit(1); } } while(0)
int main(int argc,char **argv)
{
    CHECK(argc==2);const char *mode=argv[1];
    HomeConfig c={.period_ms=1,.seek_timeout_ms=0,.total_timeout_ms=220000,
        .backoff={4000,1240,400},.seek_dir=4,.backoff_period_ms=1,.qualify_unexpected=true};
    HomeSequence s;
    if(!strcmp(mode,"bad_zero") || !strcmp(mode,"bad_slow")) {
        if(!strcmp(mode,"bad_zero"))c.period_ms=0;else c.backoff_period_ms=1001;
        HomeSequence_Init(&s,&c);CHECK(s.phase==HOME_FAILED && s.reason==HOME_BAD_CONFIG);return 0;
    }
    if(!strcmp(mode,"fallback"))c.backoff_period_ms=0;
    uint32_t base=!strcmp(mode,"wrap")?UINT32_MAX-50U:0;
    HomeSequence_Init(&s,&c);CHECK(s.config.backoff_period_ms==1);
    s.parallel=strcmp(mode,"single")!=0;
    HomeSequence_Tick(&s,base,0);HomeSequence_Tick(&s,base+10U,0);
    CHECK(HomeSequence_Start(&s,base+11U,0));
    for(unsigned t=12;t<=1011;t++) {
        HomeSequence_Tick(&s,base+t,0);
        CHECK(s.phase==HOME_SEEK && s.seek[0]==t-11U);
        if(s.parallel)CHECK(s.seek[1]==s.seek[0] && s.seek[2]==s.seek[0]);
        if(!strcmp(mode,"same_ms")) {
            HomeSequence_Tick(&s,base+t,0);
            CHECK(s.seek[0]==t-11U && !s.high && !s.step_mask);
        }
    }
    CHECK(s.seek[0]==1000);
    if(!strcmp(mode,"gap")) {
        HomeSequence_Tick(&s,base+4000U,0);CHECK(s.seek[0]==1001);
        HomeSequence_Tick(&s,base+4000U,0);CHECK(s.seek[0]==1001 && !s.high);
    } else if(!strcmp(mode,"first_low") || !strcmp(mode,"barrier")) {
        HomeSequence_Tick(&s,base+1012U,1);CHECK(s.seek[0]==1000 && !(s.step_mask&1));
        for(unsigned t=1013;t<=1022;t++)HomeSequence_Tick(&s,base+t,1);
        CHECK(s.phase==HOME_SEEK && s.contacts==1 && !s.back[0]);
        if(!strcmp(mode,"barrier")) {
            for(unsigned t=1023;t<=1033;t++)HomeSequence_Tick(&s,base+t,7);
            CHECK(s.phase==HOME_BACKOFF && s.contacts==7 && !s.step_mask);
            HomeSequence_Tick(&s,base+1034U,7);
            CHECK(s.step_mask==7 && s.back[0]==1 && s.back[1]==1 && s.back[2]==1);
        }
    } else if(!strcmp(mode,"deadline")) {
        HomeSequence_Tick(&s,base+220011U,0);
        CHECK(s.phase==HOME_FAILED && s.reason==HOME_TOTAL_TIMEOUT && !s.enable && !s.high);
    } else if(!strcmp(mode,"cancel")) {
        HomeSequence_Fail(&s,HOME_CANCELLED);HomeSequence_Tick(&s,base+1012U,0);
        CHECK(s.phase==HOME_FAILED && !s.enable && !s.high && !s.step_mask && s.seek[0]==1000);
    }
    printf("home1000 %s passed\n",mode);return 0;
}
