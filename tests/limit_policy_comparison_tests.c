#include "ctrl_motion_engine.h"
#include "waveform_service.h"
#include "fake_robot.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    /* 相同3ms输入：正常运动的10ms复读过滤，波形诊断的原始边沿立即锁存停止。
     * 编译真实生产模块，说明软件策略差异本身就能产生不同结果。
     * 这不验证板上瞬态来源，也不宣称编译或运行过原始备份固件。
     */
    FakeRobot_Reset();Ctrl_MotionEngine_Init();
    Ctrl_MotionEngine_EnableLimitMonitoring(true);
    MotionFrame_t frame={.delta_m1=20,.total_ticks=20};
    assert(Ctrl_MotionEngine_PushFrame(&frame));
    fake_limits[2]=true;Ctrl_MotionEngine_NotifyLimitSwitch(MOTION_LIMIT_M3);
    Ctrl_MotionEngine_OnStepTickFromISR();assert(fake_steps[0]==0);
    FakeRobot_AdvanceUs(3000);fake_limits[2]=false;
    Ctrl_MotionEngine_ServiceSafety();assert(!Ctrl_MotionEngine_HasFault());
    FakeRobot_AdvanceUs(7000);Ctrl_MotionEngine_ServiceSafety();
    Ctrl_MotionEngine_OnStepTickFromISR();
    assert(!Ctrl_MotionEngine_HasFault() && fake_steps[0]==1);

    FakeRobot_Reset();WaveformService_Init();WaveformService_Feed(true);
    assert(WaveformService_Start(1,0,false,100,10));
    for(unsigned i=0;i<100;i++) WaveformService_OnTickFromISR();
    fake_limits[2]=true;WaveformService_OnLimitFromISR(2);
    FakeRobot_AdvanceUs(3000);fake_limits[2]=false;
    WaveformService_Feed(true);
    for(unsigned i=0;i<1000;i++) WaveformService_OnTickFromISR();
    WaveformStatus_t wave;WaveformService_GetStatus(&wave);
    assert(wave.state==WAVE_LIMIT && wave.emitted==1 && wave.limit_mask==4);
    assert(wave.limit_edges[2]==1 && !fake_wave_level[0] && fake_enable_true_count==0);
    puts("limit_policy_comparison_tests: passed");return 0;
}
