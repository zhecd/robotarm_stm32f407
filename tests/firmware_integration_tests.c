/** 实际 App/Service/Algorithm 集成，仅设备、时间、串口端口使用替身。 */
#include "app_init.h"
#include "app/app_serial_protocol.h"
#include "app/app_teleop.h"
#include "motion_service.h"
#include "command_service.h"
#include "safety_service.h"
#include "service/svc_homing.h"
#include "fake_hardware.h"
#include "fake_robot.h"
#include "robot_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void Send(const char *line) { FakeHardware_ClearTx(); FakeHardware_SendLine(line); App_RunOnce(); }
static void CompleteHoming(void)
{
    uint32_t start_steps=fake_steps[0];
    bool injected=false;
    for(unsigned i=0;i<3;i++) fake_limits[i]=true;
    FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US); App_RunOnce();
    for(unsigned i=0;i<3;i++) fake_limits[i]=false;
    for(unsigned i=0;i<HOMING_MAX_STEPS*4U && !SafetyService_IsMotionAllowed();i++) {
        if (!injected && fake_steps[0]==start_steps+(uint32_t)(HOMING_BACKOFF_M1_DEG*UNITS_PER_DEGREE)) {
            FakeHardware_SendLine("G1 X5"); FakeHardware_SendLine("G1 X6");
            injected=true;
        }
        FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US); App_RunOnce();
    }
    assert(SafetyService_IsMotionAllowed());
    App_RunOnce(); assert(!CommandService_IsMotionPending());
}
int main(void)
{
    FakeRobot_Reset(); FakeHardware_Reset(); App_Teleop_SetMode(SYS_MODE_GCODE);
    App_Init();
    assert(fake_normal_init_count==1 && fake_diagnostic_init_count==0);
    assert(!fake_joints_enabled && fake_gripper_init_count==0);
    App_RunOnce(); /* 三路反馈就绪后才进入原有自动回零流程。 */
    assert(fake_configure_count==1 && fake_joints_enabled);
    assert(fake_gripper_init_count==1);
    Send("M115"); assert(strstr(fake_tx,"PROTOCOL:1"));
    Send("G1 X10"); assert(strstr(fake_tx,"system not ready"));
    CompleteHoming(); assert(fake_timer_start_count==1);
    Send("M408"); assert(strstr(fake_tx,"STATE:READY HOMED:1 ALLOWED:1"));
    Send("M114"); assert(strstr(fake_tx,"M114 PLAN X:"));
    Send("M119"); assert(strstr(fake_tx,"M1:OPEN M2:OPEN M3:OPEN"));
    Send("M3"); assert(strstr(fake_tx,"M3OK") && fake_gripper_open_count==1);
    Send("M5"); assert(strstr(fake_tx,"M5OK") && fake_gripper_close_count==1);
    Send("M999"); assert(strstr(fake_tx,"requires a safety fault"));
    App_Teleop_SetMode(SYS_MODE_PS2);
    Send("G1 X10"); assert(strstr(fake_tx,"PS2 mode"));
    Send("M408"); assert(strstr(fake_tx,"MODE:PS2"));
    App_Teleop_SetMode(SYS_MODE_GCODE);

    MotionFrame_t frame={100,0,0,10000};
    assert(MotionService_SubmitFrame(&frame));
    Send("M400"); assert(fake_tx[0]=='\0');
    Send("M114"); assert(strstr(fake_tx,"M114 PLAN")); /* 等待完成时仍可查询。 */
    Send("M112"); assert(strstr(fake_tx,"M112OK"));
    assert(MotionService_HasFault() && SafetyService_GetFault()==SAFETY_FAULT_HOST_STOP);
    assert(MotionService_IsIdle() && !CommandService_IsMotionPending());
    unsigned steps=fake_steps[0];
    for(unsigned i=0;i<10000;i++) MotionService_OnStepTickFromISR();
    assert(fake_steps[0]==steps);
    Send("G1 X10"); assert(strstr(fake_tx,"error:"));
    Send("M999"); assert(strstr(fake_tx,"homing started"));
    Send("M112"); assert(strstr(fake_tx,"M112OK") && !fake_joints_enabled);
    Send("M999"); assert(strstr(fake_tx,"homing started"));
    CompleteHoming();
    Send("M408"); assert(strstr(fake_tx,"STATE:READY HOMED:1 ALLOWED:1"));

    /* 模拟电机准时到位，端到端验证串口等待与闭环判定。 */
    Send("G1 X1 F1000");
    for(unsigned i=0;i<200000 && CommandService_IsMotionPending();i++) {
        App_RunOnce();
        FakeRobot_AdvanceUs(2000);
        for(unsigned tick=0;tick<100;tick++) MotionService_OnStepTickFromISR();
    }
    assert(!CommandService_IsMotionPending());
    Send("M400");
    for(unsigned i=0;i<200000 && !strstr(fake_tx,"ok\r\n");i++) {
        for(unsigned tick=0;tick<100;tick++) MotionService_OnStepTickFromISR();
        if (MotionService_IsIdle()) {
            ClosedLoopStatus_t status;
            MotionService_GetClosedLoopStatus(&status);
            memcpy(fake_motor_deg, status.target_motor_deg, sizeof(fake_motor_deg));
        }
        FakeRobot_AdvanceUs(2000);
        App_RunOnce();
    }
    assert(strstr(fake_tx,"ok\r\n"));
    Send("M114"); assert(strstr(fake_tx,"X:1.00"));
    puts("firmware_integration_tests: passed"); return 0;
}
