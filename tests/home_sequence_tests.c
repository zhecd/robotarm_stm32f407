/** 真实纯状态机：覆盖接触、逐轴退回、恢复回执和启动触点暂停。
 * 这些测试只验证算法合同，不宣称机械碰撞余量或电气脉宽已实测。 */
#include "common/home_sequence.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static HomeConfig cfg={20,30000,220000,{5,4,3},4,0,false};
static void Init(HomeSequence *s,uint8_t initial,uint32_t now)
{
    HomeSequence_Init(s,&cfg);HomeSequence_Tick(s,now,initial);
    s->mask_request=0;HomeSequence_Tick(s,now+10U,initial);
    assert(HomeSequence_BootReady(s,initial));assert(HomeSequence_Start(s,now+11U,initial));
}
static void Ack(HomeSequence *s)
{for(uint8_t a=0;a<3;a++)if(s->rearm_request&(1U<<a))HomeSequence_AckRearm(s,a,true);s->mask_request=0;}
static uint8_t Inputs(const HomeSequence *s,uint8_t initial)
{
    uint8_t low=(uint8_t)(initial&~s->completed);
    uint8_t bit=(uint8_t)(1U<<s->axis);
    if(s->phase==HOME_SEEK && s->seek[s->axis]>=3U)low|=bit;
    if(s->phase==HOME_CONTACT || (s->phase==HOME_BACKOFF && s->back[s->axis]<2U))low|=bit;
    if((s->phase==HOME_BACKOFF && s->back[s->axis]>=2U) || s->phase==HOME_RELEASE)low&=(uint8_t)~bit;
    return low;
}
int main(int argc,char **argv)
{
    assert(argc==2);const char *mode=argv[1];HomeSequence s;
    if(!strcmp(mode,"sequence") || !strcmp(mode,"pressed") || !strcmp(mode,"full_scale")){
        uint8_t initial=!strcmp(mode,"pressed")?7:0;
        if(!strcmp(mode,"full_scale")){cfg.backoff[0]=4000;cfg.backoff[1]=1240;cfg.backoff[2]=200;}
        Init(&s,initial,0);unsigned last_axis=0;
        for(uint32_t t=12;t<220000 && s.phase!=HOME_DONE && s.phase!=HOME_FAILED;t++){
            uint8_t low=Inputs(&s,initial);HomeSequence_Tick(&s,t,low);Ack(&s);
            assert(s.axis>=last_axis);last_axis=s.axis;
            if(s.high)assert(s.enable && (s.phase==HOME_SEEK || s.phase==HOME_BACKOFF));
            if(s.phase==HOME_BACKOFF)assert(s.dir_high==((cfg.seek_dir&(1U<<s.axis))==0));
        }
        assert(s.phase==HOME_DONE && s.contacts==7 && s.completed==7 && s.rearmed==7 && !s.owned);
        for(unsigned a=0;a<3;a++){
            assert(s.back[a]==cfg.backoff[a]);assert(s.seek[a]==(initial?0U:3U));
        }
        assert(!s.enable && !s.high && !HomeSequence_Start(&s,220001,0));
    }else if(!strcmp(mode,"contact_bounce")){
        Init(&s,0,0);assert(HomeSequence_OnEdge(&s,0,20,1));
        assert(s.phase==HOME_CONTACT && !s.enable && !s.high);
        HomeSequence_Tick(&s,21,0);HomeSequence_Tick(&s,25,1);HomeSequence_Tick(&s,34,1);
        assert(s.phase==HOME_CONTACT && s.seek[0]==0);
        HomeSequence_Tick(&s,35,1);assert(s.phase==HOME_BACKOFF && s.contacts==1);
    }else if(!strcmp(mode,"unstable")){
        Init(&s,0,0);assert(HomeSequence_OnEdge(&s,0,20,1));
        for(unsigned t=21;t<=120;t++)HomeSequence_Tick(&s,t,(t%2)?1:0);
        assert(s.phase==HOME_FAILED && s.reason==HOME_UNSTABLE && s.contacts==0 && !s.enable);
    }else if(!strcmp(mode,"seek_timeout")){
        Init(&s,0,0);HomeSequence_Tick(&s,30011,0);
        assert(s.reason==HOME_SEEK_TIMEOUT && !s.high && !s.enable);
    }else if(!strcmp(mode,"total_timeout")){
        cfg.total_timeout_ms=100;Init(&s,0,0);HomeSequence_Tick(&s,111,0);
        assert(s.reason==HOME_TOTAL_TIMEOUT);
    }else if(!strcmp(mode,"stuck") || !strcmp(mode,"no_ack") || !strcmp(mode,"rearm_denied")){
        Init(&s,0,0);assert(HomeSequence_OnEdge(&s,0,20,1));
        for(unsigned t=21;t<700 && s.phase!=HOME_FAILED;t++){
            uint8_t low=(!strcmp(mode,"stuck") || s.phase==HOME_CONTACT)?1:0;
            HomeSequence_Tick(&s,t,low);
            if(s.rearm_request && !strcmp(mode,"rearm_denied"))HomeSequence_AckRearm(&s,0,false);
        }
        assert(s.phase==HOME_FAILED && !s.completed && !s.enable);
        assert(s.reason==(!strcmp(mode,"rearm_denied")?HOME_PORT:HOME_BACK_TIMEOUT));
    }else if(!strcmp(mode,"unexpected")){
        Init(&s,0,0);HomeSequence_Tick(&s,20,2);assert(s.reason==HOME_UNEXPECTED && !s.enable);
    }else if(!strcmp(mode,"hold") || !strcmp(mode,"hold_release")){
        Init(&s,2,0);HomeSequence_Tick(&s,31,2);assert(s.seek[0]==1);
        HomeSequence_Tick(&s,32,0);assert(s.holding && !s.high && !s.enable);
        if(!strcmp(mode,"hold_release")){
            HomeSequence_Tick(&s,42,0);assert(s.reason==HOME_UNEXPECTED);
        }else{
            for(unsigned t=33;t<43;t++){HomeSequence_Tick(&s,t,2);assert(s.seek[0]==1 && !s.high);}
            HomeSequence_Tick(&s,43,2);assert(!s.holding && s.hold_count==1 && s.seek[0]==1);
            /* 32→43暂停11ms；保留此前已等待的1ms，下一步应在31+20+11=62。 */
            HomeSequence_Tick(&s,61,2);assert(s.seek[0]==1 && !s.high);
            HomeSequence_Tick(&s,62,2);assert(s.seek[0]==2 && s.high);
            HomeSequence_Tick(&s,63,2);assert(s.seek[0]==2 && !s.high);
        }
    }else if(!strcmp(mode,"retrigger")){
        Init(&s,0,0);HomeSequence_OnEdge(&s,0,20,1);HomeSequence_Tick(&s,30,1);
        HomeSequence_Tick(&s,31,0);HomeSequence_Tick(&s,41,0);Ack(&s);
        assert(s.rearmed==1 && !s.owned);
        HomeSequence_OnEdge(&s,0,42,1);assert(s.reason==HOME_UNEXPECTED && !s.enable);
    }else if(!strcmp(mode,"boot_glitch")){
        HomeSequence_Init(&s,&cfg);HomeSequence_OnEdge(&s,1,1,2);HomeSequence_Tick(&s,11,2);
        assert(HomeSequence_BootReady(&s,2));HomeSequence_Tick(&s,12,0);
        assert(!HomeSequence_BootReady(&s,2));HomeSequence_Tick(&s,13,2);HomeSequence_Tick(&s,23,2);
        assert(HomeSequence_BootReady(&s,2));
    }else if(!strcmp(mode,"wrap")){
        Init(&s,0,0xfffffff0U);HomeSequence_Tick(&s,15U,0);assert(s.seek[0]==1 && s.high);
        HomeSequence_Tick(&s,16U,0);assert(!s.high && s.seek[0]==1);
    }else if(!strcmp(mode,"first_cause")){
        Init(&s,0,0);HomeSequence_Fail(&s,HOME_IRQ);HomeSequence_Fail(&s,HOME_CANCELLED);
        HomeSequence_AckRearm(&s,0,true);HomeSequence_Tick(&s,10000,0);
        assert(s.reason==HOME_IRQ && !s.high && !s.enable);
    }else if(!strcmp(mode,"recovery_release")){
        HomeSequence_PrepareRecovery(&s,&cfg,2,100);
        HomeSequence_Tick(&s,100,0);HomeSequence_Tick(&s,109,0);
        assert(!s.rearm_request && !HomeSequence_BootReady(&s,0));
        HomeSequence_Tick(&s,110,0);assert(s.rearm_request==2);
        HomeSequence_AckRearm(&s,1,true);assert(HomeSequence_BootReady(&s,0) && !s.rearmed);
        assert(HomeSequence_Start(&s,111,0));
    }else if(!strcmp(mode,"config")){
        cfg.period_ms=0;HomeSequence_Init(&s,&cfg);assert(s.reason==HOME_BAD_CONFIG);
    }else assert(!"unknown scenario");
    printf("home_sequence %s passed\n",mode);return 0;
}
