/** 寻限位50/退回500微步每秒：真实算法、完整步数、脉冲间隔及截止。
 * 时间由测试驱动，不代表负载转矩、实际机械角度或仪器测得的脉宽。 */
#include "common/home_sequence.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(int argc,char **argv)
{
    assert(argc==2);const char *mode=argv[1];
    HomeConfig c={20,30000,220000,{4000,1240,200},4,2,false};HomeSequence s;
    if(!strcmp(mode,"bad1001")){
        c.backoff_period_ms=1001U;
        HomeSequence_Init(&s,&c);assert(s.reason==HOME_BAD_CONFIG && !s.enable);return 0;
    }
    if(!strcmp(mode,"one_ms"))c.backoff_period_ms=1U;
    if(!strcmp(mode,"fallback")){
        c.backoff_period_ms=0;HomeSequence_Init(&s,&c);
        assert(s.config.backoff_period_ms==20 && s.reason==HOME_OK);return 0;
    }
    uint32_t base=!strcmp(mode,"wrap")?UINT32_MAX-100U:0U;
    uint8_t initial=!strcmp(mode,"pressed")?7U:!strcmp(mode,"pause")?2U:0U;
    HomeSequence_Init(&s,&c);HomeSequence_Tick(&s,base,initial);
    HomeSequence_Tick(&s,base+10,initial);assert(HomeSequence_Start(&s,base+11,initial));
    uint32_t last[3]={base+11,0,0},back_began[3]={0},duration[3]={0};
    for(uint32_t age=12;age<15000 && s.phase!=HOME_DONE && s.phase!=HOME_FAILED;age++){
        uint8_t low=(uint8_t)(initial&~s.completed);
        uint8_t a=s.axis,bit=(uint8_t)(1U<<a);HomePhase previous=s.phase;
        if((previous==HOME_SEEK && s.seek[a]>=3) || previous==HOME_CONTACT ||
           (previous==HOME_BACKOFF && s.back[a]<2))low|=bit;
        if((previous==HOME_BACKOFF && s.back[a]>=2) || previous==HOME_RELEASE)low&=(uint8_t)~bit;
        if(!strcmp(mode,"timeout") && (previous==HOME_BACKOFF || previous==HOME_RELEASE))low|=bit;
        /* M2启动压住时偶发1ms松开，验证暂停后不补发相邻脉冲。 */
        if(!strcmp(mode,"pause") && a==0 && previous==HOME_BACKOFF && age%500U==100U)low&=(uint8_t)~2U;
        uint32_t old_seek=s.seek[a],old_back=s.back[a];
        HomeSequence_Tick(&s,base+age,low);
        if(previous!=s.phase && s.phase==HOME_BACKOFF){back_began[a]=base+age;last[a]=base+age;}
        if(previous!=s.phase && s.phase==HOME_SEEK)last[s.axis]=base+age;
        if(s.seek[a]!=old_seek){
            assert(s.high && s.seek[a]==old_seek+1 && (uint32_t)(base+age-last[a])>=20U);
            last[a]=base+age;
        }
        if(s.back[a]!=old_back){
            assert(s.high && s.back[a]==old_back+1 && (uint32_t)(base+age-last[a])>=c.backoff_period_ms);
            last[a]=base+age;
            if(s.back[a]==c.backoff[a])duration[a]=(uint32_t)(base+age-back_began[a]);
        }
        if(s.rearm_request)for(uint8_t axis=0;axis<3;axis++)
            if(s.rearm_request&(1U<<axis))HomeSequence_AckRearm(&s,axis,true);
    }
    if(!strcmp(mode,"timeout")){
        assert(s.reason==HOME_BACK_TIMEOUT && s.phase==HOME_FAILED && !s.enable);
        assert(s.back[0]==4000 && duration[0]==8000);
    }else{
        assert(s.phase==HOME_DONE && s.completed==7 && !s.enable && !s.high);
        for(unsigned a=0;a<3;a++){
            assert(s.back[a]==c.backoff[a]);
            if(!strcmp(mode,"pause") && a==0)assert(duration[a]>8000 && s.hold_count>0);
            else assert(duration[a]==c.backoff[a]*c.backoff_period_ms);
        }
    }
    printf("home speed %s: M1/M2/M3 backoff %lu/%lu/%lu ms\n",mode,
           (unsigned long)duration[0],(unsigned long)duration[1],(unsigned long)duration[2]);
    return 0;
}
