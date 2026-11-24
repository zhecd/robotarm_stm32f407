/** 真实App与协议审计构建：驱动/编码器检查通过也必须停在首个动作之前。 */
#include "app_init.h"
#include "app/app_teleop.h"
#include "safety_service.h"
#include "fake_hardware.h"
#include "fake_robot.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);return 1; } } while (0)
static unsigned ready_service_calls;
void __wrap_App_GCodeExec_Service(void) { ++ready_service_calls; }
static void Send(const char *s)
{ FakeHardware_ClearTx();FakeHardware_SendLine(s);App_RunOnce(); }
static bool Inactive(void)
{
    return !fake_joints_enabled && !fake_gripper_init_count && !fake_timer_start_count &&
        !fake_steps[0] && !fake_steps[1] && !fake_steps[2] && !SafetyService_IsMotionAllowed();
}
int main(int argc,char **argv)
{
    const char *mode=argc>1?argv[1]:"normal";
    FakeRobot_Reset();FakeHardware_Reset();App_Teleop_SetMode(SYS_MODE_GCODE);
    if (!strcmp(mode,"driver")) fake_driver_prepare_ok=false;
    if (!strcmp(mode,"timeout") || !strcmp(mode,"stop")) fake_encoder_failure[1]=true;
    App_Init();App_RunOnce();CHECK(Inactive());
    Send("M115");CHECK(strstr(fake_tx,"PROTOCOL:AUDIT1") && strstr(fake_tx,"MOTOR_ENABLE:LOCKED"));
#if ROBOTARM_IRQ_EVIDENCE_MODE
    CHECK(strstr(fake_tx,"GRIPPER_STATUS,IRQ_EVIDENCE\r\n"));
#endif
    if (!strcmp(mode,"normal")) {
        Send("M408");CHECK(strstr(fake_tx,"STATE:OUTPUT_LOCKED HOMED:0 ALLOWED:0"));
        CHECK(!SafetyService_HasFault());
        Send("M999");CHECK(strstr(fake_tx,"requires a safety fault"));
        Send("G1 X1");CHECK(strstr(fake_tx,"system not ready"));
        Send("M3");CHECK(strstr(fake_tx,"system not ready"));
    } else if (!strcmp(mode,"driver")) {
        CHECK(SafetyService_GetFault()==SAFETY_FAULT_DRIVER_CONFIG);
        CHECK(fake_encoder_reads==0);
        Send("M999");CHECK(Inactive() && fake_configure_count==2);
        fake_driver_prepare_ok=true;
        Send("M999");CHECK(Inactive());
        Send("M408");CHECK(strstr(fake_tx,"STATE:OUTPUT_LOCKED") && strstr(fake_tx,"FAULT:10"));
        Send("M999");CHECK(strstr(fake_tx,"already in progress"));
    } else {
        Send("M408");CHECK(strstr(fake_tx,"STATE:CHECKING"));
        if (!strcmp(mode,"timeout")) {
            FakeRobot_AdvanceUs(10000000);App_RunOnce();
            CHECK(SafetyService_GetFault()==SAFETY_FAULT_ENCODER);
        } else {
            Send("M112");CHECK(SafetyService_GetFault()==SAFETY_FAULT_HOST_STOP);
        }
        fake_encoder_failure[1]=false;
        App_RunOnce();Send("M408");CHECK(strstr(fake_tx,"STATE:FAILED"));
        Send("M999");Send("M408");CHECK(strstr(fake_tx,"STATE:OUTPUT_LOCKED"));
    }
    CHECK(Inactive() && !SafetyService_IsMotionAllowed());
    Send("M112");Send("M408");CHECK(strstr(fake_tx,"STATE:FAILED"));
    SafetyFault_t fault=SafetyService_GetFault();
    Send("M999");Send("M408");CHECK(strstr(fake_tx,"STATE:OUTPUT_LOCKED HOMED:0 ALLOWED:0"));
    CHECK(SafetyService_GetFault()==fault && Inactive());
    FakeRobot_AdvanceUs(30000000);App_RunOnce();CHECK(Inactive());
    CHECK(ready_service_calls==0); /* 动作边界不能落入READY任务调度。 */
    printf("startup_audit_tests (%s): passed; no homing simulated\n",mode);return 0;
}
