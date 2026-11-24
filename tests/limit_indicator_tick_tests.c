/** 真实生产1ms路由：GPIO未就绪时不更新，故障停机后仍持续显示，不产生动作。 */
#include "app_isr_adapter.h"
#include "fake_hardware.h"
#include "fake_robot.h"
#include "motion_service.h"
#include "state_service.h"
#include "safety_service.h"
#include <assert.h>
#include <stdio.h>
void Dev_LimitSwitch_MaskInterrupt(DevJointId_t a) { fake_limit_irq_mask|=(uint8_t)(1U<<a); }
bool Dev_LimitSwitch_TryRearmInterrupt(DevJointId_t a)
{ if(fake_limits[a])return false;fake_limit_irq_mask&=(uint8_t)~(1U<<a);return true; }
void AppRuntime_RequestModeSwitch(void) { assert(false); }
int main(void)
{
    FakeRobot_Reset();FakeHardware_Reset();StateService_Init();SafetyService_Init();MotionService_Init();
    App_ISR_OnMillisecond();assert(fake_limit_indicator_updates==0);
    App_ISR_OnLimitsConfigured();
    for(unsigned i=0;i<8;i++) {
        fake_limits[0]=(i&1U)!=0;fake_limits[1]=(i&2U)!=0;fake_limits[2]=(i&4U)!=0;
        FakeRobot_AdvanceUs(1000);App_ISR_OnMillisecond();
        assert(fake_limit_indicator_updates==i+1U);
    }
    MotionService_AbortForFault(MOTION_FAULT_HOST_STOP);
    for(unsigned i=0;i<20;i++){FakeRobot_AdvanceUs(1000);App_ISR_OnMillisecond();}
    assert(fake_limit_indicator_updates==28 && !fake_steps[0] && !fake_steps[1] && !fake_steps[2]);
    assert(!fake_gripper_init_attempts && !fake_gripper_open_count && !fake_gripper_close_count);
    puts("limit_indicator_tick_tests: passed");return 0;
}
