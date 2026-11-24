/** 编码器启动检查的异步故障边界测试。
 * 编译真实App/安全/过载服务；仅时间、设备与通信为电脑替身。
 * 显式注入过载锁存用于检验处理顺序，不模拟最初的电气触发来源。
 */
#include "app_init.h"
#include "app/app_teleop.h"
#include "safety_service.h"
#include "limit_irq_guard.h"
#include "robot_config.h"
#include "fake_hardware.h"
#include "fake_robot.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr,"feedback check line %d: %s\n",__LINE__,#x);exit(1); } } while(0)
static const char *scenario;
static unsigned selected;
static bool armed,injected;

bool __real_AppHardware_ReadMotorAngle(uint8_t axis,float *angle);
bool __wrap_AppHardware_ReadMotorAngle(uint8_t axis,float *angle)
{
    bool result=__real_AppHardware_ReadMotorAngle(axis,angle);
    if(armed && !injected && axis==selected) {
        if(!strcmp(scenario,"deadline"))
            FakeRobot_AdvanceUs(STARTUP_ENCODER_TIMEOUT_MS*1000U);
        injected=true;
        for(unsigned i=0;i<LIMIT_IRQ_GUARD_BURST_LIMIT;i++)
            (void)LimitIrqGuard_OnEdgeFromISR(axis);
        CHECK(LimitIrqGuard_GetLatchMask()==(1U<<axis));
        AppHardware_StopActuators(); /* 对应真实ISR锁存时的即时输出停止。 */
    }
    return result;
}
static void Send(const char *line)
{
    FakeHardware_ClearTx();FakeHardware_SendLine(line);App_RunOnce();
}
int main(int argc,char **argv)
{
    CHECK(argc==3);scenario=argv[1];selected=(unsigned)strtoul(argv[2],NULL,10);CHECK(selected<3);
    CHECK(!strcmp(scenario,"valid") || !strcmp(scenario,"invalid") ||
          !strcmp(scenario,"deadline") || !strcmp(scenario,"recovery"));
    FakeRobot_Reset();FakeHardware_Reset();App_Teleop_SetMode(SYS_MODE_GCODE);App_Init();
    bool recovery=!strcmp(scenario,"recovery");
    if(recovery) {
        Send("M112");CHECK(SafetyService_GetFault()==SAFETY_FAULT_HOST_STOP);
        CHECK(fake_encoder_reads==0);
    }
    if(!strcmp(scenario,"invalid"))fake_encoder_failure[selected]=true;
    armed=true;
    if(recovery)Send("M999");else App_RunOnce();
    SafetyFault_t expected=recovery?SAFETY_FAULT_HOST_STOP:SAFETY_FAULT_LIMIT_IRQ_OVERLOAD;
    printf("observed scenario=%s axis=%u reads=%u fault=%u expected=%u injected=%u\n",
           scenario,selected,fake_encoder_reads,SafetyService_GetFault(),expected,(unsigned)injected);
    CHECK(injected && SafetyService_GetFault()==expected);
    /* 锁存后本轮就停止读取，避免随后两个I2C有限阻塞继续消耗时间。 */
    CHECK(fake_encoder_reads==selected+1U);
    CHECK(!fake_joints_enabled && !fake_gripper_init_attempts && !fake_timer_start_count);
    CHECK(!SafetyService_IsMotionAllowed() && !fake_steps[0] && !fake_steps[1] && !fake_steps[2]);
    unsigned reads=fake_encoder_reads,prepares=fake_configure_count;
    for(unsigned i=0;i<10;i++) {
        Send("M408");CHECK(strstr(fake_tx,"STATE:FAILED HOMED:0 ALLOWED:0"));
        Send("M999");CHECK(strstr(fake_tx,"limit IRQ overload"));
        Send("M112");CHECK(strstr(fake_tx,"M112OK"));
        FakeRobot_AdvanceUs(100000U);App_RunOnce();
        CHECK(SafetyService_GetFault()==expected);
        CHECK(fake_encoder_reads==reads && fake_configure_count==prepares);
        CHECK(!fake_joints_enabled && !fake_gripper_init_attempts && !fake_timer_start_count);
        CHECK(LimitIrqGuard_GetLatchMask()==(1U<<selected));
    }
    puts("feedback IRQ: immediate failure, first fault preserved, repeated M999/M112 cannot unlock");
    return 0;
}
