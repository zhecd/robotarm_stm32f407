/** 回零完成提交边界：真实App/标定/回零/服务，设备与时间为电脑替身。
 * 注入的是已知过载刺激；不模拟原始输入波形或实际NVIC/APB。
 * 若测试点处于临界区，刺激只排队，在最外层恢复中断后交付。
 */
#include "app_init.h"
#include "app/app_teleop.h"
#include "app/app_calibration.h"
#include "app/app_serial_protocol.h"
#include "service/svc_homing.h"
#include "motion_service.h"
#include "safety_service.h"
#include "limit_irq_guard.h"
#include "platform_critical.h"
#include "robot_config.h"
#include "fake_hardware.h"
#include "fake_robot.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if(!(x)){fprintf(stderr,"finalize check line %d: %s\n",__LINE__,#x);exit(1);} } while(0)
static const char *point;
static unsigned axis,ready_requests,timer_requests,calibrations;
static bool armed,pending,injected;
static void Deliver(void)
{
    if(!pending || FakeRobot_CriticalDepth()!=0U)return;
    pending=false;injected=true;
    for(unsigned i=0;i<LIMIT_IRQ_GUARD_BURST_LIMIT;i++)
        (void)LimitIrqGuard_OnEdgeFromISR((uint8_t)axis);
    CHECK(LimitIrqGuard_GetLatchMask()==(1U<<axis));
    AppHardware_StopActuators();
}
static void Inject(const char *here)
{
    if(!armed || injected || pending || strcmp(point,here))return;
    pending=true;Deliver();
}
void __real_PlatformCritical_Exit(PlatformCriticalState_t key);
void __wrap_PlatformCritical_Exit(PlatformCriticalState_t key)
{
    __real_PlatformCritical_Exit(key);Deliver();
}
bool __real_App_Calibration_Execute(void);
bool __wrap_App_Calibration_Execute(void)
{
    CHECK(FakeRobot_CriticalDepth()==0U); /* I2C标定不得搬入关中断区域。 */
    bool ok=__real_App_Calibration_Execute();CHECK(ok);
    ++calibrations;armed=true;
    Inject("calibration_ok");Inject("calibration_fail");
    return ok && strcmp(point,"calibration_fail") && strcmp(point,"plain_failure");
}
void __real_SafetyService_MarkHomed(void);
void __wrap_SafetyService_MarkHomed(void)
{ __real_SafetyService_MarkHomed();Inject("reference_mark"); }
void __real_SafetyService_ClearAfterSuccessfulHoming(void);
void __wrap_SafetyService_ClearAfterSuccessfulHoming(void)
{ __real_SafetyService_ClearAfterSuccessfulHoming();Inject("reference_clear"); }
void __real_MotionService_ClearFault(void);
void __wrap_MotionService_ClearFault(void)
{ __real_MotionService_ClearFault();Inject("motion_clear"); }
void __real_MotionService_SetLimitMonitoring(bool enabled);
void __wrap_MotionService_SetLimitMonitoring(bool enabled)
{ __real_MotionService_SetLimitMonitoring(enabled);if(enabled)Inject("monitor"); }
void __real_AppHardware_StartStepTimer(void);
void __wrap_AppHardware_StartStepTimer(void)
{ ++timer_requests;__real_AppHardware_StartStepTimer();Inject("timer"); }
void __real_AppHardware_DiscardRx(void);
void __wrap_AppHardware_DiscardRx(void)
{ __real_AppHardware_DiscardRx();Inject("discard"); }
void __real_AppSerial_SetMode(AppSerialMode_t mode);
void __wrap_AppSerial_SetMode(AppSerialMode_t mode)
{ if(mode==APP_SERIAL_READY)++ready_requests;__real_AppSerial_SetMode(mode); }
static void Send(const char *line)
{ FakeHardware_ClearTx();FakeHardware_SendLine(line);App_RunOnce(); }

int main(int argc,char **argv)
{
    CHECK(argc==3 || argc==4);point=argv[1];axis=(unsigned)strtoul(argv[2],NULL,10);CHECK(axis<3);
    bool recovery=argc==4;CHECK(!recovery || !strcmp(argv[3],"recovery"));
    FakeRobot_Reset();FakeHardware_Reset();App_Teleop_SetMode(SYS_MODE_GCODE);App_Init();
    if(recovery) {
        Send("M112");CHECK(SafetyService_GetFault()==SAFETY_FAULT_HOST_STOP);
        Send("M999");
    } else App_RunOnce();
    CHECK(Svc_Homing_GetState()==SVC_HOMING_SEEK);
    /* 实际运行有限回零模型：三开关触发后释放，保留生产退回步数。 */
    for(unsigned i=0;i<3;i++)fake_limits[i]=true;
    FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US);App_RunOnce();
    CHECK(Svc_Homing_GetState()==SVC_HOMING_BACKOFF);
    for(unsigned i=0;i<3;i++)fake_limits[i]=false;
    for(unsigned n=0;n<4500 && calibrations==0;n++) {
        FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US);App_RunOnce();
    }
    CHECK(calibrations==1 && !pending && FakeRobot_CriticalDepth()==0U);
    printf("observed point=%s axis=%u injected=%u fault=%u allowed=%u ready_requests=%u timer_requests=%u\n",
           point,axis,(unsigned)injected,SafetyService_GetFault(),
           (unsigned)SafetyService_IsMotionAllowed(),ready_requests,timer_requests);
    if(!strcmp(point,"success")) {
        CHECK(!injected && ready_requests==1 && timer_requests==1 && SafetyService_IsMotionAllowed());
        Send("M408");CHECK(strstr(fake_tx,"STATE:READY HOMED:1 ALLOWED:1"));return 0;
    }
    if(!strcmp(point,"plain_failure")) {
        CHECK(!injected && SafetyService_GetFault()==SAFETY_FAULT_INTERNAL);
    } else {
        CHECK(injected && SafetyService_GetFault()==(recovery?SAFETY_FAULT_HOST_STOP:SAFETY_FAULT_LIMIT_IRQ_OVERLOAD));
        CHECK(LimitIrqGuard_GetLatchMask()==(1U<<axis));
    }
    CHECK(!SafetyService_IsMotionAllowed() && ready_requests==0 && !fake_joints_enabled);
    if(strcmp(point,"timer") && strcmp(point,"discard"))CHECK(timer_requests==0);
    unsigned steps[3]={fake_steps[0],fake_steps[1],fake_steps[2]};
    Send("M408");CHECK(strstr(fake_tx,"STATE:FAILED HOMED:0 ALLOWED:0"));
    if(strcmp(point,"plain_failure")) {
        for(unsigned n=0;n<5;n++) {
            Send("M999");CHECK(strstr(fake_tx,"limit IRQ overload"));Send("M112");
            CHECK(!fake_joints_enabled && ready_requests==0 && calibrations==1);
            for(unsigned i=0;i<3;i++)CHECK(fake_steps[i]==steps[i]);
        }
    }
    puts("startup finalization: no false READY, no success permission, latch retained");return 0;
}
