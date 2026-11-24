/** 真实遥操作/规划/运动/闭环组合：目标同步、有界短帧、松杆和输入丢失。 */
#include "app_init.h"
#include "app/app_teleop.h"
#include "app/app_runtime_events.h"
#include "command_service.h"
#include "motion_service.h"
#include "safety_service.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include "robot_config.h"
#include "robot_math.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);return 1; } } while(0)
static bool Ready(void)
{
    FakeRobot_Reset();FakeHardware_Reset();App_Teleop_SetMode(SYS_MODE_GCODE);App_Init();App_RunOnce();
    for(unsigned i=0;i<3;i++)fake_limits[i]=true;
    FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US);App_RunOnce();
    for(unsigned i=0;i<3;i++)fake_limits[i]=false;
    for(unsigned i=0;i<HOMING_MAX_STEPS*4U && !SafetyService_IsMotionAllowed();i++) {
        FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US);App_RunOnce();
    }
    App_Teleop_SetMode(SYS_MODE_PS2);return SafetyService_IsMotionAllowed();
}
static bool Drain(void)
{
    for(unsigned i=0;i<200000 && !MotionService_IsIdle();i++)MotionService_OnStepTickFromISR();
    int32_t a,b,c;MotionService_GetTheorySteps(&a,&b,&c);
    fake_motor_deg[0]=StepsToDeg(a);fake_motor_deg[1]=StepsToDeg(b);fake_motor_deg[2]=StepsToDeg(c);
    return MotionService_IsIdle();
}
static void Send(const char *s) { FakeHardware_ClearTx();FakeHardware_SendLine(s);App_RunOnce(); }
int main(int argc,char **argv)
{
    CHECK(argc==2 && Ready());const char *scenario=argv[1];
    if(!strcmp(scenario,"mode_switch_recovery")) {
        App_Teleop_SetMode(SYS_MODE_GCODE);fake_motor_deg[0]=10.0f;
        FakeRobot_AdvanceUs(500000);MotionService_UpdateClosedLoop();
        CHECK(MotionService_IsClosedLoopRecoveryActive());
        /* 只消耗预填队列，暂不调用闭环任务；制造ISR排空与主循环续填之间的窗口。 */
        for(unsigned i=0;i<200000 && !MotionService_IsIdle();i++)MotionService_OnStepTickFromISR();
        CHECK(MotionService_IsIdle() && MotionService_IsClosedLoopRecoveryActive());
        AppRuntime_RequestModeSwitch();App_RunOnce();
        CHECK(App_Teleop_GetMode()==SYS_MODE_GCODE);
    } else if(!strcmp(scenario,"target")) {
        for(unsigned i=0;i<20;i++) { CHECK(CommandService_RunTeleopStep(0.9f,0,0)==ERR_OK);CHECK(Drain()); }
        for(unsigned i=0;i<10;i++) {
            FakeRobot_AdvanceUs((CL_SETTLE_MS+20U)*1000U);MotionService_UpdateClosedLoop();
            CHECK(!MotionService_IsClosedLoopRecoveryActive() && MotionService_GetQueueCount()==0);
        }
    } else {
        CHECK(CommandService_RunTeleopStep(0.9f,0,0)==ERR_OK);
        if(!strcmp(scenario,"single")) {
            MotionService_OnStepTickFromISR();CHECK(MotionService_IsRunning());
            CHECK(CommandService_RunTeleopStep(0.9f,0,0)==ERR_OK);
            CHECK(MotionService_GetQueueCount()==1);
            CHECK(CommandService_RunTeleopStep(0.9f,0,0)==ERR_BUSY);
        } else if(!strcmp(scenario,"neutral")) {
            fake_ps2_valid=true;
            FakeRobot_AdvanceUs(PS2_POLL_INTERVAL_MS*1000U);App_Teleop_Task();
            CHECK(MotionService_GetQueueCount()==1); /* 已受理短帧完成，不能丢帧后保留旧规划参考。 */
            CHECK(Drain());CHECK(!SafetyService_HasFault());
        } else if(!strcmp(scenario,"loss") || !strcmp(scenario,"loss_recovery") ||
                  !strcmp(scenario,"loss_recovery_held")) {
            MotionService_OnStepTickFromISR();CHECK(MotionService_IsRunning());
            uint32_t before[3]={fake_steps[0],fake_steps[1],fake_steps[2]};
            fake_ps2_valid=false;
            FakeRobot_AdvanceUs(PS2_POLL_INTERVAL_MS*1000U);App_RunOnce();
            CHECK(MotionService_HasFault() && SafetyService_HasFault());
            for(unsigned i=0;i<20000;i++)MotionService_OnStepTickFromISR();
            CHECK(fake_steps[0]==before[0] && fake_steps[1]==before[1] && fake_steps[2]==before[2]);
            CHECK(!SafetyService_IsMotionAllowed() && !CommandService_IsMotionPending());
            CHECK(MotionService_GetFaultReason()==MOTION_FAULT_INPUT_LOST);
            CHECK(SafetyService_GetFault()==SAFETY_FAULT_INPUT_LOST);
            if(strcmp(scenario,"loss")) {
                fake_ps2_valid=true;fake_ps2_input.left_x=255;fake_ps2_input.square_pressed=true;
                FakeRobot_AdvanceUs(20000);App_RunOnce();
                CHECK(!SafetyService_IsMotionAllowed() && !fake_gripper_open_count);
                CHECK(fake_steps[0]==before[0] && fake_steps[1]==before[1] && fake_steps[2]==before[2]);
                fake_ps2_input=(AppTeleopInput_t){128,128,128,false,false};
                Send("M999");CHECK(strstr(fake_tx,"homing started"));
                for(unsigned i=0;i<3;i++)fake_limits[i]=true;
                FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US);App_RunOnce();
                for(unsigned i=0;i<3;i++)fake_limits[i]=false;
                for(unsigned i=0;i<HOMING_MAX_STEPS*4U && !SafetyService_IsMotionAllowed();i++) {
                    FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US);App_RunOnce();
                }
                CHECK(SafetyService_IsMotionAllowed() && !MotionService_HasFault());
                if(!strcmp(scenario,"loss_recovery_held")) {
                    fake_ps2_input.left_x=255;fake_ps2_input.square_pressed=true;
                    FakeRobot_AdvanceUs(20000);App_RunOnce();
                    CHECK(MotionService_IsIdle() && !fake_gripper_open_count);
                    fake_ps2_input=(AppTeleopInput_t){128,128,128,false,false};
                    FakeRobot_AdvanceUs(20000);App_RunOnce();
                    fake_ps2_input.square_pressed=true;
                    FakeRobot_AdvanceUs(20000);App_RunOnce();
                    CHECK(fake_gripper_open_count==1);
                }
            }
        } else { CHECK(false); }
    }
    printf("teleop_lifecycle_tests (%s): passed\n",scenario);return 0;
}
