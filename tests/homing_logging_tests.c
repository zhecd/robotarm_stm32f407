/** 真实正常入口回零：状态日志不得运行在关闭中断的临界区中。 */
#include "app_init.h"
#include "app/app_teleop.h"
#include "safety_service.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include "robot_config.h"
#include <stdio.h>
static unsigned masked_logs;
int __wrap_printf(const char *format,...)
{ (void)format;if(FakeRobot_CriticalDepth()) ++masked_logs;return 0; }
int __wrap_puts(const char *text)
{ (void)text;if(FakeRobot_CriticalDepth()) ++masked_logs;return 0; }
int main(void)
{
    FakeRobot_Reset();FakeHardware_Reset();App_Teleop_SetMode(SYS_MODE_GCODE);
    App_Init();App_RunOnce();
    for(unsigned i=0;i<3;i++) fake_limits[i]=true;
    FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US);App_RunOnce();
    for(unsigned i=0;i<3;i++) fake_limits[i]=false;
    for(unsigned i=0;i<HOMING_MAX_STEPS*4U && !SafetyService_IsMotionAllowed();i++) {
        FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US);App_RunOnce();
    }
    if(masked_logs || !SafetyService_IsMotionAllowed()) {
        fprintf(stderr,"FAIL: %u log calls inside homing critical section; allowed=%d\n",
                masked_logs,SafetyService_IsMotionAllowed());return 1;
    }
    fprintf(stderr,"homing_logging_tests: passed\n");return 0;
}
