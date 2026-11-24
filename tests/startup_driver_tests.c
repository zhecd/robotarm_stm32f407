/** 真实App/协议/安全服务：驱动配置未确认时，编码器正常也不得启动。 */
#include "app_init.h"
#include "app/app_teleop.h"
#include "service/svc_homing.h"
#include "safety_service.h"
#include "fake_hardware.h"
#include "fake_robot.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);return 1; } } while (0)
static void Send(const char *s)
{ FakeHardware_ClearTx();FakeHardware_SendLine(s);App_RunOnce(); }
static bool Inactive(void)
{
    return !fake_joints_enabled && !fake_gripper_init_count && !fake_timer_start_count &&
           !fake_steps[0] && !fake_steps[1] && !fake_steps[2];
}
int main(void)
{
    FakeRobot_Reset();FakeHardware_Reset();App_Teleop_SetMode(SYS_MODE_GCODE);
    fake_driver_prepare_ok=false; /* 编码器默认全部有效，失败来自驱动确认。 */
    App_Init();App_RunOnce();
    CHECK(Inactive() && fake_configure_count==1 && fake_encoder_reads==0);
    CHECK(SafetyService_GetFault()==SAFETY_FAULT_DRIVER_CONFIG);
    Send("M408");CHECK(strstr(fake_tx,"STATE:FAILED") && strstr(fake_tx,"FAULT:10"));
    Send("M115");CHECK(strstr(fake_tx,"PROTOCOL:1"));
    Send("M119");CHECK(strstr(fake_tx,"M1:OPEN M2:OPEN M3:OPEN"));
    Send("G1 X1");CHECK(strstr(fake_tx,"system not ready") && Inactive());
    Send("M3");CHECK(strstr(fake_tx,"system not ready") && Inactive());
    for (unsigned i=0;i<30;i++) { FakeRobot_AdvanceUs(1000000);App_RunOnce(); }
    CHECK(fake_configure_count==1 && Inactive()); /* 不自动重试。 */
    Send("M999");CHECK(fake_configure_count==2 && Inactive());
    Send("M112");CHECK(Inactive() && SafetyService_GetFault()==SAFETY_FAULT_DRIVER_CONFIG);
    fake_driver_prepare_ok=true;
    App_RunOnce();CHECK(Inactive() && fake_configure_count==2); /* 通信恢复不能自行启动。 */
    /* 驱动恢复后编码器尚未就绪；首故障仍为驱动，M112仍必须取消本轮恢复。 */
    fake_encoder_failure[1]=true;
    Send("M999");CHECK(Inactive());
    Send("M112");CHECK(Inactive());
    fake_encoder_failure[1]=false;
    App_RunOnce();CHECK(Inactive());
    Send("M999");
    CHECK(fake_joints_enabled && fake_gripper_init_count==1);
    CHECK(Svc_Homing_GetState()==SVC_HOMING_SEEK && !SafetyService_IsMotionAllowed());
    CHECK(SafetyService_GetFault()==SAFETY_FAULT_DRIVER_CONFIG); /* 回零成功才清首故障。 */
    Send("M112");CHECK(!fake_joints_enabled);
    puts("startup_driver_tests: passed");return 0;
}
