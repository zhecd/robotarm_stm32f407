#include "app_init.h"
#include "app/app_teleop.h"
#include "fake_hardware.h"
#include "fake_robot.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void)
{
    FakeRobot_Reset(); FakeHardware_Reset(); App_Teleop_SetMode(SYS_MODE_GCODE);
    App_Init();
    assert(fake_diagnostic_init_count==1 && fake_normal_init_count==0);
    assert(fake_configure_count==0 && fake_timer_start_count==0 && !fake_joints_enabled);
    const char *rejected[]={"G1 X10", "M3", "M5", "M999", "M112"};
    for(unsigned i=0;i<sizeof(rejected)/sizeof(rejected[0]);i++) {
        FakeHardware_ClearTx(); FakeHardware_SendLine(rejected[i]); App_RunOnce();
        assert(strstr(fake_tx,"diagnostic mode is read-only"));
    }
    FakeHardware_ClearTx(); FakeHardware_SendLine("M408"); App_RunOnce();
    assert(strstr(fake_tx,"STATE:DIAGNOSTIC HOMED:0 ALLOWED:0"));
    fake_motor_deg[0]=12.5f;
    FakeRobot_AdvanceUs(20000); App_RunOnce();
    FakeHardware_ClearTx(); FakeHardware_SendLine("M114"); App_RunOnce();
    assert(strstr(fake_tx,"RAWENC:12.50") && strstr(fake_tx,"VALID:1,1,1"));
    assert(fake_steps[0]==0 && fake_steps[1]==0 && fake_steps[2]==0);
    assert(fake_gripper_open_count==0 && fake_gripper_close_count==0);
    puts("diagnostic_runtime_tests: passed"); return 0;
}
