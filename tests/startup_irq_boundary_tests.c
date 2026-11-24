/** 实际App/回零/安全服务：在设备调用返回和回零入口注入ISR过载锁存。
 * 只在电脑测试进程注入，设备全部是替身；不产生真实IRQ，不复现电气来源。
 */
#include "app_init.h"
#include "app/app_teleop.h"
#include "service/svc_homing.h"
#include "safety_service.h"
#include "limit_irq_guard.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include "robot_config.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* 负对照直接返回失败，避免Windows CRT断言对话框拖延故障注入测试。 */
#undef assert
#define assert(x) do {if(!(x)){fprintf(stderr,"boundary check line %d: %s\n",__LINE__,#x);exit(1);}}while(0)

static const char *point;
static uint8_t axis;
static bool injected;
static unsigned enable_requests,home_starts;
static void Inject(const char *here)
{
    if(injected || strcmp(point,here))return;
    injected=true;
    for(unsigned i=0;i<LIMIT_IRQ_GUARD_BURST_LIMIT;i++)(void)LimitIrqGuard_OnEdgeFromISR(axis);
    assert(LimitIrqGuard_GetLatchMask()==(1U<<axis));
    AppHardware_StopActuators(); /* 模拟真实ISR在锁存时立即关闭输出，安全状态交由App处理。 */
}
bool __real_AppHardware_TryInitGripper(void);
bool __wrap_AppHardware_TryInitGripper(void)
{
    bool result=__real_AppHardware_TryInitGripper();
    Inject("gripper_return");return result;
}
void __real_AppHardware_EnableJoints(bool enabled);
void __wrap_AppHardware_EnableJoints(bool enabled)
{
    if(enabled)enable_requests++;
    __real_AppHardware_EnableJoints(enabled);
    if(enabled)Inject("enable_return");
}
bool __real_Svc_Homing_Start(void);
bool __wrap_Svc_Homing_Start(void)
{
    home_starts++;Inject("home_start");return __real_Svc_Homing_Start();
}
void __real_Svc_Homing_Step(void);
void __wrap_Svc_Homing_Step(void)
{
    Inject("home_step");__real_Svc_Homing_Step();
}
int main(int argc,char **argv)
{
    assert(argc==3);point=argv[1];axis=(uint8_t)strtoul(argv[2],NULL,10);assert(axis<3);
    FakeRobot_Reset();FakeHardware_Reset();App_Teleop_SetMode(SYS_MODE_GCODE);App_Init();
    App_RunOnce();
    if(!strcmp(point,"home_step")){
        assert(!injected && Svc_Homing_GetState()==SVC_HOMING_SEEK);
        FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US+1U);App_RunOnce();
    }
    printf("observed point=%s axis=%u injected=%u safety=%u enable_requests=%u home_starts=%u\n",
           point,axis,(unsigned)injected,(unsigned)SafetyService_GetFault(),enable_requests,home_starts);
    assert(injected && SafetyService_GetFault()==SAFETY_FAULT_LIMIT_IRQ_OVERLOAD);
    assert(!SafetyService_IsMotionAllowed() && !fake_joints_enabled && !fake_timer_start_count);
    assert(!fake_steps[0] && !fake_steps[1] && !fake_steps[2]);
    if(!strcmp(point,"gripper_return"))assert(enable_requests==0 && home_starts==0);
    if(!strcmp(point,"enable_return"))assert(enable_requests==1 && home_starts==0);
    if(!strcmp(point,"home_start"))assert(home_starts==1);
    FakeHardware_ClearTx();FakeHardware_SendLine("M408");App_RunOnce();
    assert(strstr(fake_tx,"STATE:FAILED HOMED:0 ALLOWED:0") && strstr(fake_tx,"FAULT:8 MOTION_FAULT:7"));
    unsigned requests=enable_requests,starts=home_starts;
    FakeHardware_ClearTx();FakeHardware_SendLine("M999");App_RunOnce();
    assert(strstr(fake_tx,"limit IRQ overload") && enable_requests==requests && home_starts==starts);
    assert(LimitIrqGuard_GetLatchMask()==(1U<<axis));
    printf("startup boundary %s axis%u: IRQ first cause, no downstream startup or steps, no online recovery\n",point,axis+1U);
    return 0;
}
