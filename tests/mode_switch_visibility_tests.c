/** 实际生产App/ISR/回零服务：检查SW7切换、故障和M999的模式显示合同。 */
#include "app_init.h"
#include "app_isr_adapter.h"
#include "app/app_teleop.h"
#include "service/svc_homing_sequence.h"
#include "motion_service.h"
#include "safety_service.h"
#include "fake_hardware.h"
#include "fake_robot.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
void Dev_LimitSwitch_MaskInterrupt(DevJointId_t a){fake_limit_irq_mask|=(uint8_t)(1U<<a);}
bool Dev_LimitSwitch_TryRearmInterrupt(DevJointId_t a)
{if(fake_limits[a])return false;fake_limit_irq_mask&=(uint8_t)~(1U<<a);return true;}
static void CompleteHome(void)
{
    for(unsigned t=0;t<20000 && !SafetyService_IsMotionAllowed();t++) {
        SvcHomingSequenceStatus s;Svc_HomingSequence_GetStatus(&s);
        for(unsigned a=0;a<3;a++) {
            fake_limits[a]=s.flow.phase==HOME_SEEK?s.flow.seek[a]>=3U+3U*a:
                s.flow.phase==HOME_BACKOFF?s.flow.back[a]<2U:false;
        }
        FakeRobot_AdvanceUs(1000);App_ISR_OnMillisecond();App_RunOnce();
    }
    CHECK(SafetyService_IsMotionAllowed());
}
static void Key(void){App_ISR_OnGpioEdge(KEY_MODE_Pin);App_RunOnce();}
int main(int argc,char **argv)
{
    CHECK(argc==2);const char *name=argv[1];
    FakeRobot_Reset();FakeHardware_Reset();App_Teleop_SetMode(SYS_MODE_GCODE);
    App_ISR_OnLimitsConfigured();App_Init();CompleteHome();
    fake_ps2_valid=true;CHECK(fake_indicator_gcode && !fake_indicator_ps2);
    if(!strcmp(name,"switch_pair")) {
        Key();CHECK(App_Teleop_GetMode()==SYS_MODE_PS2 && fake_indicator_ps2 && !fake_indicator_gcode);
        FakeRobot_AdvanceUs(50000);Key();CHECK(App_Teleop_GetMode()==SYS_MODE_PS2);
        FakeRobot_AdvanceUs(250000);Key();CHECK(App_Teleop_GetMode()==SYS_MODE_GCODE && fake_indicator_gcode && !fake_indicator_ps2);
    } else if(!strcmp(name,"fault_mode")) {
        MotionService_AbortForFault(MOTION_FAULT_HOST_STOP);App_RunOnce();
        CHECK(!SafetyService_IsMotionAllowed() && App_Teleop_GetMode()==SYS_MODE_GCODE);
        CHECK(fake_indicator_gcode && !fake_indicator_ps2);
    } else if(!strcmp(name,"recovery_indicator")) {
        Key();CHECK(App_Teleop_GetMode()==SYS_MODE_PS2);
        FakeHardware_SendLine("M112");App_RunOnce();CHECK(!SafetyService_IsMotionAllowed());
        FakeHardware_SendLine("M999");App_RunOnce();CompleteHome();
        CHECK(App_Teleop_GetMode()==SYS_MODE_PS2 && fake_indicator_ps2 && !fake_indicator_gcode);
    } else if(!strcmp(name,"moving_refused")) {
        MotionFrame_t f={.delta_m1=1,.total_ticks=10000};CHECK(MotionService_SubmitFrame(&f));
        Key();CHECK(App_Teleop_GetMode()==SYS_MODE_GCODE && fake_indicator_gcode);
    } else CHECK(false);
    puts("mode switch visibility passed");return 0;
}
