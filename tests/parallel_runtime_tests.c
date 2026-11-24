/** 真实App/Service/限位状态机：仅设备与时钟替换，不连接板卡。 */
#include "app_init.h"
#include "app_isr_adapter.h"
#include "service/svc_homing_sequence.h"
#include "safety_service.h"
#include "motion_service.h"
#include "app/app_serial_protocol.h"
#include "fake_hardware.h"
#include "fake_robot.h"
#include "robot_config.h"
#include "platform_time.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
/* fake GPIO端口：只在稳定高、未锁存时确认释放，不冒充真实中断。 */
void Dev_LimitSwitch_MaskInterrupt(DevJointId_t a){fake_limit_irq_mask|=(uint8_t)(1U<<a);}
bool Dev_LimitSwitch_TryRearmInterrupt(DevJointId_t a)
{if(fake_limits[a])return false;fake_limit_irq_mask&=(uint8_t)~(1U<<a);return true;}
static SvcHomingSequenceStatus Status(void)
{SvcHomingSequenceStatus s;Svc_HomingSequence_GetStatus(&s);return s;}
/* 记录真实Service对设备端口的低→高调用；仍让原设备替身统计有效上升沿。
 * 证明调用顺序与毫秒步间隔，不把电脑端调用顺序当成电气脉宽测量。 */
static bool low_written[3];
static uint32_t seek_first[3],seek_last[3],back_first[3],back_last[3],seek_rises[3],back_rises[3];
bool __real_Dev_Joint_WriteStepLevel(DevJointId_t axis,bool high);
bool __wrap_Dev_Joint_WriteStepLevel(DevJointId_t axis,bool high)
{
    bool ok=__real_Dev_Joint_WriteStepLevel(axis,high);
    if(!ok)return false;
    if(!high){low_written[axis]=true;return true;}
    assert(low_written[axis]);low_written[axis]=false;
    SvcHomingSequenceStatus s=Status();uint32_t now=PlatformTime_NowMs();
    uint32_t *first=s.flow.phase==HOME_SEEK?seek_first:back_first;
    uint32_t *last=s.flow.phase==HOME_SEEK?seek_last:back_last;
    uint32_t *count=s.flow.phase==HOME_SEEK?seek_rises:back_rises;
    if(!count[axis])first[axis]=now;
    else assert((uint32_t)(now-last[axis])>=1U);
    last[axis]=now;count[axis]++;return true;
}
int main(int argc,char **argv)
{
    assert(argc==2);const char *name=argv[1];FakeRobot_Reset();FakeHardware_Reset();
    if(!strcmp(name,"pressed"))fake_limits[1]=true;
    /* 模拟CubeMX完成GPIO配置后真实发生的通知，AppHardware替身不代做此事。 */
    App_ISR_OnLimitsConfigured();App_Init();bool saw_overlap=false,saw_back=false;uint32_t max_seek[3]={0};
    for(unsigned t=0;t<20000 && !SafetyService_IsMotionAllowed();t++){
        SvcHomingSequenceStatus before=Status();
        for(uint8_t a=0;a<3;a++){
            bool low=(!strcmp(name,"pressed") && a==1 &&
                (before.flow.phase==HOME_BOOT || before.flow.phase==HOME_SEEK));
            uint32_t seek_limit=!strcmp(name,"rate1000")?1000U:3U+3U*a;
            if(before.flow.phase==HOME_SEEK && before.flow.seek[a]>=seek_limit)low=true;
            if(before.flow.phase==HOME_BACKOFF)low=before.flow.back[a]<2U || (!strcmp(name,"stuck") && a==2);
            fake_limits[a]=low;
        }
        if(!strcmp(name,"port") && before.flow.started)fake_step_failure[1]=true;
        FakeRobot_AdvanceUs(1000);App_ISR_OnMillisecond();App_RunOnce();
        SvcHomingSequenceStatus s=Status();assert(s.flow.parallel);
        assert(FakeRobot_CriticalDepth()==0);
        if(s.flow.phase==HOME_SEEK){
            if(s.flow.step_mask==7)saw_overlap=true;
            assert(s.flow.back[0]==0 && s.flow.back[1]==0 && s.flow.back[2]==0);
            for(unsigned a=0;a<3;a++)max_seek[a]=s.flow.seek[a];
        }
        if(s.flow.phase==HOME_BACKOFF){saw_back=true;assert(s.flow.contacts==7);assert(fake_joints_enabled);}
        if(s.flow.phase==HOME_FAILED)break;
    }
    SvcHomingSequenceStatus s=Status();
    if(!strcmp(name,"port") || !strcmp(name,"stuck")){
        assert(s.flow.phase==HOME_FAILED && !fake_joints_enabled);
        for(unsigned a=0;a<3;a++)assert(!fake_wave_level[a]);
        assert(!SafetyService_IsMotionAllowed());
    }else{
        if(!SafetyService_IsMotionAllowed())fprintf(stderr,"parallel runtime failure phase=%u reason=%u contacts=%u completed=%u safety=%u back=%lu,%lu,%lu\n",
            s.flow.phase,s.flow.reason,s.flow.contacts,s.flow.completed,SafetyService_GetFault(),
            (unsigned long)s.flow.back[0],(unsigned long)s.flow.back[1],(unsigned long)s.flow.back[2]);
        assert(SafetyService_IsMotionAllowed() && s.committed && s.flow.completed==7 && saw_back);
        if(strcmp(name,"pressed"))assert(saw_overlap);
        assert(s.flow.back[0]==4040 && s.flow.back[1]==1160 && s.flow.back[2]==160);
        for(unsigned a=0;a<3;a++)assert(fake_wave_rises[a]==s.flow.seek[a]+s.flow.back[a]);
        if(!strcmp(name,"rate1000"))for(unsigned a=0;a<3;a++) {
            assert(s.flow.config.period_ms==1U && s.flow.config.backoff_period_ms==1U);
            assert(seek_rises[a]==1000U && seek_last[a]-seek_first[a]==999U);
            assert(back_rises[a]==s.flow.back[a] && back_last[a]-back_first[a]==s.flow.back[a]-1U);
        }
        assert(max_seek[0]>0 && max_seek[2]>0);
        FakeHardware_ClearTx();FakeHardware_SendLine("M126");App_RunOnce();
        assert(strstr(fake_tx,"PARALLEL:1") && strstr(fake_tx,"CONTACTS:7 COMPLETE:7"));
        if(!strcmp(name,"recovery")){
            FakeHardware_SendLine("M112");App_RunOnce();assert(!fake_joints_enabled);
            FakeHardware_SendLine("M999");App_RunOnce();
            assert(Status().flow.parallel && !Status().committed);
        }
    }
    printf("parallel runtime %s passed\n",name);return 0;
}
