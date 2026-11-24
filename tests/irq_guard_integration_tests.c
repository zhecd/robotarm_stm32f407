/** 编译真实入口、协议、ISR和运动/波形服务；在设备边界注入密集限位回调。 */
#include "app_init.h"
#include "app_isr_adapter.h"
#include "app/app_teleop.h"
#include "limit_irq_guard.h"
#include "motion_service.h"
#include "safety_service.h"
#include "waveform_service.h"
#include "service/svc_homing.h"
#include "service/svc_gripper.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include "robot_config.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);return 1; } } while (0)
static void Send(const char *s) { FakeHardware_ClearTx();FakeHardware_SendLine(s);App_RunOnce(); }
static void Flood(void) { for (unsigned n=0;n<256;n++) App_ISR_OnGpioEdge(M2_STOP_Pin); }
int main(int argc,char **argv)
{
    const char *mode=argc>1?argv[1]:"checking";
    FakeRobot_Reset();FakeHardware_Reset();App_Teleop_SetMode(SYS_MODE_GCODE);
    if (!strcmp(mode,"early")) Flood();
    if (!strcmp(mode,"checking")) fake_encoder_failure[1]=true;
    App_Init();App_RunOnce();
#if ROBOTARM_WAVEFORM_MODE
    CHECK(WaveformService_Start(1,0,true,10,50));
    for (unsigned n=0;n<100;n++) WaveformService_OnTickFromISR();
    CHECK(fake_wave_level[0]);
    Flood();
    CHECK(!fake_wave_level[0]); /* 第一条原始边沿仍立即停波，不等过载阈值。 */
#else
    if (!strcmp(mode,"ready")) {
        for (unsigned i=0;i<3;i++) fake_limits[i]=true;
        FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US);App_RunOnce();
        for (unsigned i=0;i<3;i++) fake_limits[i]=false;
        for (unsigned i=0;i<HOMING_MAX_STEPS*4U && !SafetyService_IsMotionAllowed();i++) {
            FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US);App_RunOnce();
        }
        CHECK(SafetyService_IsMotionAllowed());
        MotionFrame_t frame={100,0,0,100};CHECK(MotionService_SubmitFrame(&frame));
    }
    if (strcmp(mode,"early")) Flood();
#endif
    CHECK(LimitIrqGuard_GetLatchMask()==2 && fake_limit_irq_mask==2 && !fake_joints_enabled);
    uint32_t steps[3]={fake_steps[0],fake_steps[1],fake_steps[2]};
    uint32_t rises=fake_wave_rises[0];
    TIM_HandleTypeDef timer={TIM6};
    for (unsigned n=0;n<1000;n++) App_ISR_OnTimerElapsed(&timer);
    CHECK(fake_steps[0]==steps[0] && fake_steps[1]==steps[1] && fake_steps[2]==steps[2]);
    CHECK(fake_wave_rises[0]==rises);
    App_RunOnce();CHECK(SafetyService_GetFault()==SAFETY_FAULT_LIMIT_IRQ_OVERLOAD);
    Send("M408");CHECK(strstr(fake_tx,"IRQ_LATCH_MASK:2"));
#if ROBOTARM_WAVEFORM_MODE
    CHECK(strstr(fake_tx,"WAVE_STATE:9"));
    CHECK(!WaveformService_Start(2,0,true,1,50));
#else
    CHECK(strstr(fake_tx,"STATE:FAILED") && strstr(fake_tx,"ALLOWED:0"));
    Send("M999");CHECK(strstr(fake_tx,"overload latched"));
    Send("M3");CHECK(strstr(fake_tx,"system not ready"));
    Svc_Gripper_Open();Svc_Gripper_Close();
    CHECK(fake_gripper_open_count==0 && fake_gripper_close_count==0);
#endif
    Send("M112");CHECK(strstr(fake_tx,"M112OK") && LimitIrqGuard_GetLatchMask()==2);
    Send("M119");CHECK(strstr(fake_tx,"M1:OPEN M2:OPEN M3:OPEN"));
    if (!strcmp(mode,"early") || !strcmp(mode,"checking")) CHECK(fake_gripper_init_count==0);
    printf("irq_guard_integration_tests (%s): passed\n",mode);return 0;
}
