/** 真实 App/协议/服务启动回归；替身仅提供反馈、执行器、时间与通信端口。 */
#include "app_init.h"
#include "app/app_teleop.h"
#include "service/svc_homing.h"
#include "safety_service.h"
#include "platform_time.h"
#include "robot_config.h"
#include "fake_hardware.h"
#include "fake_robot.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* 显式退出便于保留修复前失败证据，避免Windows断言对话框挂起测试。 */
#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#condition);return 1; } } while (0)
static void Send(const char *line)
{ FakeHardware_ClearTx(); FakeHardware_SendLine(line); App_RunOnce(); }
static void AdvanceMs(uint32_t ms)
{ FakeRobot_AdvanceUs(ms*1000U); App_RunOnce(); }
static bool Inactive(void)
{
    return !fake_joints_enabled && fake_gripper_init_count==0 &&
           fake_timer_start_count==0 && !fake_steps[0] && !fake_steps[1] && !fake_steps[2];
}

int main(int argc, char **argv)
{
    const char *scenario = argc>1 ? argv[1] : "delayed";
    FakeRobot_Reset(); FakeHardware_Reset(); App_Teleop_SetMode(SYS_MODE_GCODE);
    if (!strcmp(scenario,"rollover"))
        FakeRobot_SetTimeUs(((uint64_t)UINT32_MAX-1000U)*1000U);
    fake_encoder_failure[1]=true;
    App_Init();
    CHECK(Inactive());
    CHECK(!SafetyService_IsMotionAllowed());
    Send("M408"); CHECK(strstr(fake_tx,"STATE:CHECKING HOMED:0 ALLOWED:0"));
    CHECK(fake_encoder_reads==3 && Inactive());

    if (!strcmp(scenario,"gripper_hold_homing")) {
        fake_encoder_failure[1]=false;AdvanceMs(100);
        CHECK(Svc_Homing_GetState()==SVC_HOMING_SEEK);
        unsigned before=fake_gripper_service_count;
        AdvanceMs(599);CHECK(fake_gripper_service_count==before+1);
        AdvanceMs(1);CHECK(fake_gripper_service_count==before+2);
        AdvanceMs(80000);CHECK(fake_gripper_service_count==before+3);
        CHECK(Svc_Homing_GetState()==SVC_HOMING_SEEK && !SafetyService_IsMotionAllowed());
    } else if (!strcmp(scenario,"delayed")) {
        Send("M115"); CHECK(strstr(fake_tx,"PROTOCOL:1"));
        Send("M119"); CHECK(strstr(fake_tx,"M1:OPEN M2:OPEN M3:OPEN"));
        Send("G1 X1"); CHECK(strstr(fake_tx,"system not ready"));
        Send("M3"); CHECK(strstr(fake_tx,"system not ready"));
        CHECK(fake_encoder_reads==3 && Inactive()); /* 未到100ms不得忙循环重试。 */
        for (unsigned i=0;i<40;i++) { AdvanceMs(100); CHECK(Inactive()); }
        fake_encoder_failure[1]=false;
        AdvanceMs(100);
        CHECK(fake_joints_enabled && fake_gripper_init_count==1);
        CHECK(Svc_Homing_GetState()==SVC_HOMING_SEEK && fake_timer_start_count==0);
        CHECK(!SafetyService_IsMotionAllowed()); /* 就绪检查不是回零/标定成功。 */
        Send("M408"); CHECK(strstr(fake_tx,"STATE:HOMING"));
    } else if (!strcmp(scenario,"gripper_failure") || !strcmp(scenario,"gripper_recovery")) {
        SvcHomingState_t before=Svc_Homing_GetState();
        fake_encoder_failure[1]=false;
        fake_gripper_init_failure=true;
        AdvanceMs(100);
        CHECK(Inactive() && SafetyService_GetFault()==SAFETY_FAULT_INTERNAL);
        CHECK(fake_gripper_init_attempts==1 && fake_enable_true_count==0);
        /* BeginStartup先取消旧回零，初态允许为FAILED；失败不得启动新回零。 */
        CHECK(Svc_Homing_GetState()==before);
        CHECK(before!=SVC_HOMING_SEEK && before!=SVC_HOMING_BACKOFF);
        AdvanceMs(1000);CHECK(Inactive() && fake_gripper_init_attempts==1);
        Send("M408");CHECK(strstr(fake_tx,"STATE:FAILED HOMED:0 ALLOWED:0"));
        Send("M3");CHECK(strstr(fake_tx,"error:") && !strstr(fake_tx,"M3OK"));
        if (!strcmp(scenario,"gripper_recovery")) {
            fake_gripper_init_failure=false;
            Send("M999");
            CHECK(fake_gripper_init_attempts==2 && fake_gripper_init_count==1);
            CHECK(fake_joints_enabled && Svc_Homing_GetState()==SVC_HOMING_SEEK);
            CHECK(SafetyService_GetFault()==SAFETY_FAULT_INTERNAL && !SafetyService_IsMotionAllowed());
            Send("M112");CHECK(!fake_joints_enabled);
        }
    } else if (!strcmp(scenario,"stop")) {
        Send("M112"); CHECK(strstr(fake_tx,"M112OK") && Inactive());
        CHECK(SafetyService_GetFault()==SAFETY_FAULT_HOST_STOP);
        fake_encoder_failure[1]=false;
        AdvanceMs(1000); CHECK(Inactive());
        Send("M408"); CHECK(strstr(fake_tx,"STATE:FAILED"));
    } else if (!strcmp(scenario,"stale")) {
        /* 轴1上轮成功不能与轴2本轮成功拼成三轴就绪。 */
        fake_encoder_failure[0]=true;fake_encoder_failure[1]=false;
        AdvanceMs(100); CHECK(Inactive());
        fake_encoder_failure[0]=false;
        AdvanceMs(100); CHECK(fake_joints_enabled && fake_gripper_init_count==1);
    } else if (!strcmp(scenario,"deadline")) {
        fake_encoder_failure[1]=false;
        fake_encoder_read_delay_us=4000000U; /* 一轮读取返回时已跨过10秒期限。 */
        AdvanceMs(100);
        CHECK(Inactive() && SafetyService_GetFault()==SAFETY_FAULT_ENCODER);
        Send("M408"); CHECK(strstr(fake_tx,"STATE:FAILED"));
    } else if (!strcmp(scenario,"nonfinite")) {
        fake_encoder_failure[1]=false;fake_motor_deg[1]=NAN;
        AdvanceMs(100); CHECK(Inactive());
        fake_motor_deg[1]=INFINITY;
        AdvanceMs(100); CHECK(Inactive());
        fake_motor_deg[1]=12;
        AdvanceMs(100); CHECK(fake_joints_enabled && fake_gripper_init_count==1);
    } else {
        CHECK(!strcmp(scenario,"timeout") || !strcmp(scenario,"recovery") || !strcmp(scenario,"rollover"));
        uint32_t start=PlatformTime_NowMs();
        AdvanceMs(STARTUP_ENCODER_TIMEOUT_MS-1U); CHECK(Inactive() && !SafetyService_HasFault());
        AdvanceMs(1); CHECK(Inactive() && SafetyService_GetFault()==SAFETY_FAULT_ENCODER);
        CHECK((uint32_t)(PlatformTime_NowMs()-start)==STARTUP_ENCODER_TIMEOUT_MS);
        Send("M408"); CHECK(strstr(fake_tx,"STATE:FAILED"));
        if (!strcmp(scenario,"recovery")) {
            Send("M999"); CHECK(strstr(fake_tx,"homing started") && Inactive());
            CHECK(SafetyService_GetFault()==SAFETY_FAULT_ENCODER);
            Send("M999"); CHECK(strstr(fake_tx,"already in progress"));
            Send("M112"); CHECK(strstr(fake_tx,"M112OK") && Inactive());
            /* 首故障仍是ENCODER，独立停止事件必须阻止随后自动开始回零。 */
            CHECK(SafetyService_GetFault()==SAFETY_FAULT_ENCODER);
            fake_encoder_failure[1]=false;
            AdvanceMs(1000); CHECK(Inactive());
            Send("M408"); CHECK(strstr(fake_tx,"STATE:FAILED"));
            Send("M999"); CHECK(fake_joints_enabled && fake_gripper_init_count==1);
            CHECK(!SafetyService_IsMotionAllowed());
            Send("M112"); CHECK(!fake_joints_enabled);
            Send("M999"); CHECK(fake_joints_enabled && fake_gripper_init_count==1);
        }
    }
    printf("startup_readiness_tests (%s): passed\n",scenario);
    return 0;
}
