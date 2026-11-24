/** 真实协议/命令/夹爪/遥操作许可回归；故障与新请求同轮到达时也不得执行。 */
#include "app_init.h"
#include "app/app_teleop.h"
#include "app/app_runtime_events.h"
#include "command_service.h"
#include "motion_service.h"
#include "safety_service.h"
#include "service/svc_gripper.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include "robot_config.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);return 1; } } while(0)
static void Send(const char *s) { FakeHardware_ClearTx();FakeHardware_SendLine(s);App_RunOnce(); }
static bool CompleteHoming(void)
{
    for(unsigned i=0;i<3;i++) fake_limits[i]=true;
    FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US);App_RunOnce();
    for(unsigned i=0;i<3;i++) fake_limits[i]=false;
    for(unsigned i=0;i<HOMING_MAX_STEPS*4U && !SafetyService_IsMotionAllowed();i++) {
        FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US);App_RunOnce();
    }
    return SafetyService_IsMotionAllowed();
}
int main(int argc,char **argv)
{
    CHECK(argc==2);const char *scenario=argv[1];
    FakeRobot_Reset();FakeHardware_Reset();App_Teleop_SetMode(SYS_MODE_GCODE);App_Init();
    if (!strcmp(scenario,"unhomed")) {
        GCodeFrame_t frame={.type=GCMD_M3};
        CHECK(CommandService_RunGCode(&frame)==ERR_BUSY);
        CHECK(fake_gripper_open_count==0);
    } else {
        App_RunOnce();CHECK(CompleteHoming());
        if (!strcmp(scenario,"gripper_start_failure")) {
            /* 协议链替换Device；真实Service/Device/driver另有HAL集成测试。 */
            fake_gripper_start_failure=true;
            Send("M3");CHECK(strstr(fake_tx,"error: command rejected (-8)") && !strstr(fake_tx,"M3OK"));
            Send("M5");CHECK(strstr(fake_tx,"error: command rejected (-8)") && !strstr(fake_tx,"M5OK"));
            CHECK(!Svc_Gripper_Open() && !Svc_Gripper_Close());
            CHECK(fake_gripper_open_count==0 && fake_gripper_close_count==0);
            /* 显式新请求可以受理；这里模拟主机操作，不是固件自动重试。 */
            fake_gripper_start_failure=false;
            Send("M3");CHECK(strstr(fake_tx,"M3OK") && fake_gripper_open_count==1);
            Send("M5");CHECK(strstr(fake_tx,"M5OK") && fake_gripper_close_count==1);
            CHECK(FakeRobot_CriticalDepth()==0);
            puts("action_permission_tests (gripper_start_failure): passed");return 0;
        } else if (!strcmp(scenario,"command_fault")) {
            SafetyService_ReportHostStop();
            GCodeFrame_t open={.type=GCMD_M3},close={.type=GCMD_M5};
            CHECK(CommandService_RunGCode(&open)==ERR_BUSY);
            CHECK(CommandService_RunGCode(&close)==ERR_BUSY);
        } else if (!strcmp(scenario,"gripper_fault")) {
            SafetyService_ReportEncoderFailure();Svc_Gripper_Open();Svc_Gripper_Close();
        } else if (!strcmp(scenario,"serial_fault")) {
            Send("M112");CHECK(strstr(fake_tx,"M112OK"));
            Send("M3");CHECK(strstr(fake_tx,"error:") && !strstr(fake_tx,"M3OK"));
            Send("M5");CHECK(strstr(fake_tx,"error:") && !strstr(fake_tx,"M5OK"));
        } else if (!strcmp(scenario,"direct_limit") || !strcmp(scenario,"direct_gripper_limit")) {
            fake_limits[0]=true;MotionService_NotifyLimitSwitch(MOTION_LIMIT_M1);FakeRobot_AdvanceUs(10000);
            if(!strcmp(scenario,"direct_limit")) {
                GCodeFrame_t frame={.type=GCMD_M3};CHECK(CommandService_RunGCode(&frame)==ERR_BUSY);
            } else { Svc_Gripper_Open(); }
            CHECK(MotionService_GetFaultReason()==MOTION_FAULT_LIMIT_SWITCH);
        } else if (!strcmp(scenario,"qualified_limit")) {
            fake_limits[0]=true;MotionService_NotifyLimitSwitch(MOTION_LIMIT_M1);
            FakeRobot_AdvanceUs(10000);Send("M3");
            CHECK(SafetyService_GetFault()==SAFETY_FAULT_LIMIT_SWITCH);
            CHECK(strstr(fake_tx,"error:") && !strstr(fake_tx,"M3OK"));
        } else if (!strncmp(scenario,"correction_",11)) {
            MotionFaultReason_t reason=!strcmp(scenario,"correction_range")?MOTION_FAULT_CORRECTION_RANGE:
                (!strcmp(scenario,"correction_timeout")?MOTION_FAULT_CORRECTION_TIMEOUT:MOTION_FAULT_CORRECTION_STALLED);
            MotionService_AbortForFault(reason);Send("M3");
            CHECK(SafetyService_GetFault()==SAFETY_FAULT_CONTROL_DIVERGENCE);
            CHECK(strstr(fake_tx,"error:") && MotionService_GetFaultReason()==reason);
            CHECK(!fake_gripper_open_count && !SafetyService_IsMotionAllowed());
        } else if (!strcmp(scenario,"motion_fault")) {
            MotionService_AbortForFault(MOTION_FAULT_ENCODER);Send("M3");
            CHECK(SafetyService_GetFault()==SAFETY_FAULT_ENCODER);
            CHECK(strstr(fake_tx,"error:") && !strstr(fake_tx,"M3OK"));
        } else if (!strcmp(scenario,"ps2_fault")) {
            App_Teleop_SetMode(SYS_MODE_PS2);SafetyService_ReportHostStop();
            fake_ps2_valid=true;fake_ps2_input.square_pressed=true;
            FakeRobot_AdvanceUs(PS2_POLL_INTERVAL_MS*1000U);App_Teleop_Task();
        } else if (!strcmp(scenario,"ps2_feedback") || !strcmp(scenario,"ps2_switch")) {
            if (!strcmp(scenario,"ps2_switch")) AppRuntime_RequestModeSwitch();
            else App_Teleop_SetMode(SYS_MODE_PS2);
            fake_encoder_failure[1]=true;
            float angle;
            for(unsigned i=1;i<CL_ENCODER_FAIL_LIMIT;i++) CHECK(!MotionService_GetClosedLoopAxisAngle(1,&angle));
            CHECK(!MotionService_HasFault());
            fake_ps2_valid=true;fake_ps2_input.square_pressed=true;fake_ps2_input.left_x=255;
            FakeRobot_AdvanceUs(20000);App_RunOnce();
            CHECK(SafetyService_GetFault()==SAFETY_FAULT_ENCODER);
            CHECK(!CommandService_IsMotionPending());
        } else if (!strcmp(scenario,"recovery_success")) {
            Send("M112");Send("M3");CHECK(strstr(fake_tx,"error:") && !fake_gripper_open_count);
            Send("M999");CHECK(CompleteHoming());
            Send("M3");CHECK(strstr(fake_tx,"M3OK") && fake_gripper_open_count==1);
            Send("M5");CHECK(strstr(fake_tx,"M5OK") && fake_gripper_close_count==1);
            puts("action_permission_tests (recovery_success): passed");return 0;
        } else if (!strcmp(scenario,"wait_reply")) {
            MotionFrame_t frame={100,0,0,10000};CHECK(MotionService_SubmitFrame(&frame));
            Send("M400");CHECK(!fake_tx[0]);
            MotionService_AbortForFault(MOTION_FAULT_ENCODER);
            FakeHardware_ClearTx();App_RunOnce();
            CHECK(strstr(fake_tx,"M400 aborted by safety fault") && !strstr(fake_tx,"ok\r\n"));
        } else { CHECK(false); }
        CHECK(fake_gripper_open_count==0 && fake_gripper_close_count==0);
    }
    printf("action_permission_tests (%s): passed\n",scenario);return 0;
}
