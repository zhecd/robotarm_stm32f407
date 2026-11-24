#include "app_init.h"
#include "app/app_diagnostics.h"
#include "limit_observation_service.h"
#include "app_isr_adapter.h"
#include "fake_hardware.h"
#include "fake_robot.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static const char *Query(const char *line)
{
    FakeHardware_ClearTx();FakeHardware_SendLine(line);App_RunOnce();return fake_tx;
}
int main(void)
{
    FakeRobot_Reset();FakeHardware_Reset();App_Init();
    assert(!AppDiagnostics_EncoderSamplingEnabled());
    assert(fake_diagnostic_init_count==1 && fake_timer_start_count==0);
    assert(strstr(Query("M115"),"PROTOCOL:DIAG3") && strstr(fake_tx,"ENCODER_POLL:0"));
    assert(strstr(fake_tx,"LIMIT_OBSERVATION,IRQ_FUSE") && fake_limit_irq_reset_count==1);
    FakeRobot_AdvanceUs(10000);App_RunOnce();
    assert(strstr(Query("M408"),"UPTIME_MS:10 SAMPLE_MS:10 ENCODER_POLL:0"));
    assert(strstr(fake_tx,"RAW_MASK:0 STABLE_MASK:0 VALID_MASK:7 EDGE:0,0,0 PRESS:0,0,0"));
    LimitObservation_OnEdgeFromISR(2);fake_limits[2]=true;App_RunOnce();
    FakeRobot_AdvanceUs(3000);fake_limits[2]=false;App_RunOnce();
    FakeRobot_AdvanceUs(10000);App_RunOnce();
    assert(strstr(Query("M408"),"EDGE:0,0,1 PRESS:0,0,0"));
    fake_limits[2]=true;LimitObservation_OnEdgeFromISR(2);App_RunOnce();
    FakeRobot_AdvanceUs(10000);App_RunOnce();
    assert(strstr(Query("M408"),"RAW_MASK:4 STABLE_MASK:4 VALID_MASK:7 EDGE:0,0,2 PRESS:0,0,1"));
    assert(strstr(Query("M119"),"M3:TRIGGERED"));
    assert(strstr(Query("M114"),"VALID:0,0,0"));
    /* 用真实入口、ISR适配器和观察服务复现不前进的时钟下连续中断。
     * 替身IMR被屏蔽后即停止投递，检验恢复的是主循环可运行性，而不是输入已正常。 */
    fake_limits[2]=false;App_RunOnce();FakeRobot_AdvanceUs(10000);App_RunOnce();
    fake_limits[1]=true;App_RunOnce();
    unsigned delivered=0;
    while ((fake_limit_irq_mask&2U)==0U && delivered<100000U) {
        App_ISR_OnGpioEdge(M2_STOP_Pin);++delivered;
    }
    assert(delivered==LIMIT_OBSERVATION_IRQ_BURST_LIMIT && fake_limit_irq_mask==2);
    FakeRobot_AdvanceUs(1000000);App_RunOnce();
    assert(strstr(Query("M408"),"IRQ_LATCH_MASK:2") && strstr(fake_tx,"STABLE_MASK:2"));
    assert(strstr(Query("M119"),"M2:TRIGGERED"));
    fake_limits[1]=false;App_RunOnce();FakeRobot_AdvanceUs(10000);App_RunOnce();
    assert(strstr(Query("M408"),"STABLE_MASK:0") && strstr(fake_tx,"IRQ_LATCH_MASK:2"));
    App_Init();assert(fake_limit_irq_mask==0 && fake_limit_irq_reset_count==2);
    assert(strstr(Query("M408"),"IRQ_LATCH_MASK:0"));
    const char *rejected[]={"WAVE I1 A1 D0 N1 F10","M122 N0","G1 X1","M3","M999","M112"};
    for(unsigned i=0;i<sizeof(rejected)/sizeof(rejected[0]);i++)
        assert(strstr(Query(rejected[i]),"diagnostic mode is read-only"));
    assert(fake_encoder_reads==0 && fake_driver_reads==0 && fake_configure_count==0);
    assert(fake_enable_true_count==0 && fake_timer_start_count==0);
    assert(fake_gripper_open_count==0 && fake_gripper_close_count==0);
    for(unsigned i=0;i<3;i++) assert(fake_steps[i]==0 && fake_wave_rises[i]==0);
    puts("diagnostic_minimal_runtime_tests: passed");return 0;
}
