/** 并行回零真实纯算法测试。替换时钟/限位输入，不模拟机械到位。 */
#include "common/home_sequence.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static HomeConfig config={.period_ms=2,.total_timeout_ms=220000,.backoff={30,20,10},
    .seek_dir=4,.backoff_period_ms=2,.qualify_unexpected=true};
static void Init(HomeSequence *s,uint8_t pressed,uint32_t now)
{
    HomeSequence_Init(s,&config);s->parallel=true;
    for(uint32_t d=0;d<=10;d++)HomeSequence_Tick(s,now+d,pressed);
    assert(HomeSequence_Start(s,now+11,pressed));
}
static void Ack(HomeSequence *s,bool deny)
{
    s->mask_request=0;
    for(uint8_t a=0;a<3;a++)if(s->rearm_request&(1U<<a))HomeSequence_AckRearm(s,a,!deny);
}
static uint8_t Model(HomeSequence *s,uint8_t initial,bool stuck)
{
    uint8_t low=initial;
    for(uint8_t a=0;a<3;a++){
        uint8_t bit=(uint8_t)(1U<<a);
        if(s->phase==HOME_SEEK && s->seek[a]>=3U+3U*a)low|=bit;
        if(s->phase==HOME_BACKOFF && (s->back[a]<2 || stuck))low|=bit;
        else if(s->phase==HOME_BACKOFF)low&=(uint8_t)~bit;
    }
    return low;
}
int main(int argc,char **argv)
{
    assert(argc==2);const char *name=argv[1];HomeSequence s;
    if(!strncmp(name,"initial",7) || !strcmp(name,"stuck") || !strcmp(name,"deny") || !strcmp(name,"scale") || !strcmp(name,"wrap")){
        uint8_t initial=!strncmp(name,"initial",7)?(uint8_t)atoi(name+7):0;
        uint32_t origin=!strcmp(name,"wrap")?UINT32_MAX-50U:0;
        if(!strcmp(name,"scale")){config.backoff[0]=4000;config.backoff[1]=1240;config.backoff[2]=400;}
        Init(&s,initial,origin);uint32_t last[3]={0};uint8_t previous=0;
        for(uint32_t t=12;t<12000 && s.phase!=HOME_DONE && s.phase!=HOME_FAILED;t++){
            bool seeking=s.phase==HOME_SEEK;uint8_t low=Model(&s,initial,!strcmp(name,"stuck"));
            HomeSequence_Tick(&s,origin+t,low);Ack(&s,!strcmp(name,"deny"));
            assert(!(s.step_mask&previous));previous=s.step_mask;
            for(uint8_t a=0;a<3;a++)if(s.step_mask&(1U<<a)){
                assert(!last[a] || t-last[a]>=2);last[a]=t;
                if(seeking)assert(!(low&(1U<<a)));
            }
            if(s.phase==HOME_SEEK){assert(s.dir_mask==4 && s.enable);assert(!s.back[0]&&!s.back[1]&&!s.back[2]);}
            if(s.phase==HOME_BACKOFF){assert(s.contacts==7 && s.dir_mask==3 && s.enable);}
        }
        if(!strcmp(name,"stuck") || !strcmp(name,"deny")){
            assert(s.phase==HOME_FAILED && !s.enable && !s.step_mask);
            assert(s.reason==(!strcmp(name,"stuck")?HOME_BACK_TIMEOUT:HOME_PORT));
        }else{
            assert(s.phase==HOME_DONE && s.completed==7 && s.rearmed==7 && !s.owned && !s.enable);
            for(unsigned a=0;a<3;a++){assert(s.back[a]==config.backoff[a]);assert(s.seek[a]==((initial&(1U<<a))?0U:3U+3U*a));}
        }
    }else if(!strcmp(name,"barrier")){
        Init(&s,0,0);
        for(uint32_t t=12;t<50;t++){HomeSequence_Tick(&s,t,3);Ack(&s,false);assert(s.phase==HOME_SEEK && !s.seek[0]&&!s.seek[1] && s.back[0]==0);}
        assert(s.contacts==3 && s.seek[2]>0);
        for(uint32_t t=50;t<=60;t++)HomeSequence_Tick(&s,t,7);
        assert(s.phase==HOME_BACKOFF && s.step_mask==0 && s.contacts==7);
        HomeSequence_Tick(&s,61,7);assert(!s.step_mask);
        HomeSequence_Tick(&s,62,7);assert(s.step_mask==7 && s.back[0]==1 && s.back[1]==1 && s.back[2]==1);
    }else if(!strcmp(name,"glitch")){
        Init(&s,0,0);HomeSequence_Tick(&s,12,1);assert(!(s.step_mask&1));
        for(uint32_t t=13;t<=23;t++)HomeSequence_Tick(&s,t,0);
        assert(s.contact_cancel_count==1 && !(s.contacts&1) && s.seek[1]>0);
        HomeSequence_Tick(&s,24,0);assert(!(s.step_mask&1));
        HomeSequence_Tick(&s,25,0);assert(s.step_mask&1);
    }else if(!strcmp(name,"gap")){
        Init(&s,0,0);HomeSequence_Tick(&s,12,7);HomeSequence_Tick(&s,24,7);
        assert(s.phase==HOME_SEEK && s.contacts==0);
        for(uint32_t t=25;t<=34;t++)HomeSequence_Tick(&s,t,7);
        assert(s.phase==HOME_BACKOFF);
    }else if(!strcmp(name,"unstable") || !strcmp(name,"confirmed_release")){
        Init(&s,0,0);
        for(uint32_t t=12;t<130 && s.phase!=HOME_FAILED;t++){
            uint8_t low=!strcmp(name,"unstable")?(t%2?1:0):(t<=25?1:0);
            HomeSequence_Tick(&s,t,low);
        }
        assert(s.phase==HOME_FAILED && !s.step_mask && !s.enable);
        assert(s.reason==(!strcmp(name,"unstable")?HOME_UNSTABLE:HOME_UNEXPECTED));
    }else if(!strcmp(name,"retrigger_glitch") || !strcmp(name,"retrigger_held") || !strcmp(name,"back_gap")){
        Init(&s,0,0);
        for(uint32_t t=12;t<=22;t++)HomeSequence_Tick(&s,t,7);
        assert(s.phase==HOME_BACKOFF);
        for(uint32_t t=23;t<=33;t++){HomeSequence_Tick(&s,t,0);Ack(&s,false);}
        assert(s.rearmed==7);
        if(!strcmp(name,"back_gap")){
            /* 新一轮重建释放状态；跨12ms无采样不能当作稳定释放。 */
            s.rearmed=0;s.owned=7;s.parallel_high_seen=0;
            HomeSequence_Tick(&s,34,0);HomeSequence_Tick(&s,46,0);assert(!s.rearm_request);
            for(uint32_t t=47;t<=56;t++)HomeSequence_Tick(&s,t,0);
            assert(s.rearm_request==7);
        }else{
            uint32_t before=s.back[0];HomeSequence_Tick(&s,34,1);
            assert(s.holding && s.enable && !s.step_mask && s.back[0]==before);
            for(uint32_t t=35;t<=45;t++)HomeSequence_Tick(&s,t,!strcmp(name,"retrigger_held")?1:0);
            if(!strcmp(name,"retrigger_held"))assert(s.phase==HOME_FAILED && s.reason==HOME_UNEXPECTED && !s.enable);
            else{
                /* 暂停前已经等满的周期可在恢复拍发一脉冲，不补发多脉冲。 */
                assert(!s.holding && s.phase==HOME_BACKOFF && s.back[0]==before+1U && s.enable);
                HomeSequence_Tick(&s,46,0);assert(s.back[0]==before+1U && !s.step_mask);
                HomeSequence_Tick(&s,47,0);assert(s.back[0]==before+2U);
            }
        }
    }else if(!strcmp(name,"deadline") || !strcmp(name,"cancel")){
        Init(&s,0,0);HomeSequence_Tick(&s,30012,0);assert(s.phase==HOME_SEEK && s.seek[0]==1);
        if(!strcmp(name,"deadline"))HomeSequence_Tick(&s,220011,0);
        else HomeSequence_Fail(&s,HOME_CANCELLED);
        assert(s.phase==HOME_FAILED && !s.step_mask && !s.enable);
        assert(s.reason==(!strcmp(name,"deadline")?HOME_TOTAL_TIMEOUT:HOME_CANCELLED));
    }else assert(!"unknown scenario");
    printf("parallel %s passed\n",name);return 0;
}
