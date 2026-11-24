#include "waveform_service.h"
#include "fake_robot.h"
#include <assert.h>
#include <stdio.h>
static WaveformStatus_t Status(void)
{
    WaveformStatus_t status; WaveformService_GetStatus(&status); return status;
}
static void Reset(void)
{
    FakeRobot_Reset(); WaveformService_Init(); WaveformService_Feed(true);
}
static void Ticks(unsigned count, bool feed)
{
    for (unsigned i=0;i<count;i++) {
        if (feed && i%500U==0) WaveformService_Feed(true);
        WaveformService_OnTickFromISR();
        assert(!fake_joints_enabled && fake_enable_true_count==0);
    }
}
int main(void)
{
    /* 精确检查 3 个轴、两个 DIR 电平、所有合法频率的边沿数与 tick 时序。 */
    const unsigned rates[]={10,20,25,50,100};
    for(unsigned axis=0;axis<3;axis++) for(unsigned dir=0;dir<2;dir++)
    for(unsigned r=0;r<5;r++) {
        Reset();
        assert(WaveformService_Start(1,(uint8_t)axis,dir!=0,100,(uint16_t)rates[r]));
        assert(fake_clockwise[axis]==(dir==0));
        unsigned period=50000U/rates[r];
        for(unsigned tick=1;tick<=100U+99U*period+5U;tick++) {
            if (tick%500U==0) WaveformService_Feed(true);
            WaveformService_OnTickFromISR();
            bool high=tick>=100U && ((tick-100U)%period)<5U;
            assert(fake_wave_level[axis]==high);
            for(unsigned other=0;other<3;other++) if (other!=axis)
                assert(fake_wave_rises[other]==0 && !fake_wave_level[other]);
            assert(!fake_joints_enabled && fake_enable_true_count==0);
        }
        assert(Status().state==WAVE_DONE && Status().emitted==100);
        assert(fake_wave_rises[axis]==100 && !fake_wave_level[axis]);
        assert(!WaveformService_Start(1,(uint8_t)axis,false,1,10));
        Ticks(6000,true); assert(fake_wave_rises[axis]==100);
    }
    Reset();
    assert(!WaveformService_Start(1,3,false,1,10));
    assert(!WaveformService_Start(1,0,false,0,10));
    assert(!WaveformService_Start(1,0,false,101,10));
    assert(!WaveformService_Start(1,0,false,1,11));
    assert(!WaveformService_Start(0,0,false,1,10));
    WaveformService_Feed(false);
    assert(!WaveformService_Start(1,0,false,1,10));
    WaveformService_Feed(true);
    for(unsigned axis=0;axis<3;axis++) {
        fake_limits[axis]=true;
        assert(!WaveformService_Start(1,0,false,1,10));
        fake_limits[axis]=false;
    }
    assert(WaveformService_Start(1,0,false,100,10));
    assert(!WaveformService_Start(2,1,false,1,10)); /* 忙时不排队 */
    Ticks(100,false); assert(fake_wave_level[0]);
    WaveformService_Stop(WAVE_HOST_STOP);
    assert(!fake_wave_level[0] && Status().state==WAVE_HOST_STOP);
    assert(!WaveformService_Start(1,0,false,1,10));
    assert(WaveformService_Start(2,1,false,1,10));
    Ticks(105,true); assert(Status().state==WAVE_DONE && fake_wave_rises[1]==1);
    for(unsigned axis=0;axis<3;axis++) {
        Reset(); assert(WaveformService_Start(1,0,false,100,10)); Ticks(100,true);
        fake_limits[axis]=true; Ticks(1,false);
        assert(Status().state==WAVE_LIMIT && !fake_wave_level[0]);
        assert(Status().limit_mask==(1U<<axis) && Status().limit_raw_mask==(1U<<axis));
        assert(Status().limit_source==2 && Status().limit_edges[axis]==0);
        fake_limits[axis]=false; Ticks(6000,true); assert(fake_wave_rises[0]==1);
    }
    Reset(); assert(WaveformService_Start(1,0,false,100,10)); Ticks(100,true);
    WaveformService_OnLimitFromISR(2);
    assert(Status().state==WAVE_LIMIT && !fake_wave_level[0]);
    assert(Status().limit_mask==4 && Status().limit_raw_mask==0 && Status().limit_source==1);
    assert(Status().limit_edges[2]==1);
    WaveformService_OnLimitFromISR(0);
    WaveformService_OnLimitFromISR(255);
    assert(Status().limit_mask==4 && Status().limit_source==1 && Status().limit_edges[0]==1);
    WaveformService_Stop(WAVE_HOST_STOP);
    assert(Status().limit_mask==4 && Status().limit_raw_mask==0); /* 软件停止不抹掉来源 */
    assert(WaveformService_Start(2,0,false,1,10));
    assert(Status().limit_mask==0 && Status().limit_raw_mask==0 && Status().limit_source==0);
    assert(Status().limit_edges[0]==1 && Status().limit_edges[2]==1); /* 新任务保留总边沿数 */
    Reset(); assert(WaveformService_Start(1,0,false,100,10)); Ticks(25000,false);
    assert(Status().state==WAVE_HEARTBEAT && !fake_wave_level[0]);
    unsigned stopped=fake_wave_rises[0]; Ticks(30000,true);
    assert(fake_wave_rises[0]==stopped);
    Reset(); assert(WaveformService_Start(1,0,false,100,10)); Ticks(100,true);
    WaveformService_Feed(false);
    assert(Status().state==WAVE_ENCODER && !fake_wave_level[0]);
    Reset(); assert(WaveformService_Start(1,0,false,1,10)); fake_step_failure[0]=true;
    Ticks(100,true); assert(Status().state==WAVE_OUTPUT);
    puts("waveform_service_tests: passed"); return 0;
}
