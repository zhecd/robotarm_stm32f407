/** 验证真实插补器：精确步数、方向、环形队列、非法帧和限位消抖暂停。 */
#include "ctrl_motion_engine.h"
#include "fake_robot.h"
#include "robot_config.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

static void Reset(void) { FakeRobot_Reset(); Ctrl_MotionEngine_Init(); }
static void Tick(unsigned n) { while (n--) Ctrl_MotionEngine_OnStepTickFromISR(); }
static void TestCountsAndWaitFrames(void)
{
    Reset();
    MotionFrame_t frame = {7, -3, 0, 20};
    MotionFrame_t wait = {0, 0, 0, 12};
    assert(Ctrl_MotionEngine_PushFrame(&frame));
    assert(Ctrl_MotionEngine_PushFrame(&wait));
    Tick(20);
    assert(fake_steps[0] == 7 && fake_steps[1] == 3 && fake_steps[2] == 0);
    assert(fake_clockwise[0] && !fake_clockwise[1]);
    Tick(11);
    assert(Ctrl_MotionEngine_IsRunning());
    Tick(1);
    assert(!Ctrl_MotionEngine_IsRunning() && Ctrl_MotionEngine_GetQueueCount() == 0);
    assert(fake_steps[0] == 7 && fake_steps[1] == 3 && fake_steps[2] == 0);
}
static void TestInvalidAndQueueWrap(void)
{
    Reset();
    MotionFrame_t frame = {1, 0, 0, 1};
    MotionFrame_t invalid[] = {{0,0,0,0}, {2,0,0,1}, {0,-2,0,1}, {0,0,INT32_MIN,1}};
    assert(!Ctrl_MotionEngine_PushFrame(NULL));
    for (unsigned i=0; i<sizeof(invalid)/sizeof(invalid[0]); ++i)
        assert(!Ctrl_MotionEngine_PushFrame(&invalid[i]));
    for (unsigned repeat=0; repeat<3; ++repeat) {
        for (unsigned i=0; i<RING_BUFFER_SIZE-1; ++i) assert(Ctrl_MotionEngine_PushFrame(&frame));
        assert(!Ctrl_MotionEngine_PushFrame(&frame));
        Tick(RING_BUFFER_SIZE-1);
        assert(Ctrl_MotionEngine_GetQueueCount() == 0);
    }
    assert(fake_steps[0] == 3*(RING_BUFFER_SIZE-1));
}
static void TestLimitPauseAndFault(void)
{
    Reset();
    MotionFrame_t frame = {20,0,0,20};
    Ctrl_MotionEngine_EnableLimitMonitoring(true);
    assert(Ctrl_MotionEngine_PushFrame(&frame));
    Tick(2);
    Ctrl_MotionEngine_NotifyLimitSwitch(MOTION_LIMIT_M1);
    Tick(4);
    assert(fake_steps[0] == 2); /* 消抖期间也不得继续推进 STEP。 */
    FakeRobot_AdvanceUs(10000);
    Ctrl_MotionEngine_ServiceSafety(); /* 抖动已消失，可恢复原帧。 */
    Tick(2);
    assert(fake_steps[0] == 4 && !Ctrl_MotionEngine_HasFault());
    fake_limits[1] = true;
    Ctrl_MotionEngine_NotifyLimitSwitch(MOTION_LIMIT_M2);
    FakeRobot_AdvanceUs(10000);
    Ctrl_MotionEngine_ServiceSafety();
    assert(Ctrl_MotionEngine_GetFaultReason() == MOTION_FAULT_LIMIT_SWITCH);
    Tick(50);
    assert(fake_steps[0] == 4 && Ctrl_MotionEngine_GetQueueCount() == 0);
    Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_QUEUE_TIMEOUT);
    assert(Ctrl_MotionEngine_GetFaultReason() == MOTION_FAULT_LIMIT_SWITCH);
    assert(!Ctrl_MotionEngine_PushFrame(&frame));
    Ctrl_MotionEngine_ClearFault();
    assert(Ctrl_MotionEngine_PushFrame(&frame));
}
static void TestIndependentAxisDebounce(void)
{
    /* M1持续按下到10ms应确认；9ms时的M3事件不能重置M1的确认时间。 */
    Reset();Ctrl_MotionEngine_EnableLimitMonitoring(true);
    fake_limits[0]=true;
    Ctrl_MotionEngine_NotifyLimitSwitch(MOTION_LIMIT_M1);
    FakeRobot_AdvanceUs(9000);
    Ctrl_MotionEngine_NotifyLimitSwitch(MOTION_LIMIT_M3);
    FakeRobot_AdvanceUs(1000);
    Ctrl_MotionEngine_ServiceSafety();
    assert(Ctrl_MotionEngine_GetFaultReason()==MOTION_FAULT_LIMIT_SWITCH);

    /* 同一路的新边沿仍重启本轴10ms窗口，保留原有消抖语义。 */
    Reset();Ctrl_MotionEngine_EnableLimitMonitoring(true);fake_limits[0]=true;
    Ctrl_MotionEngine_NotifyLimitSwitch(MOTION_LIMIT_M1);
    FakeRobot_AdvanceUs(9000);Ctrl_MotionEngine_NotifyLimitSwitch(MOTION_LIMIT_M1);
    FakeRobot_AdvanceUs(1000);Ctrl_MotionEngine_ServiceSafety();
    assert(!Ctrl_MotionEngine_HasFault());
    FakeRobot_AdvanceUs(9000);Ctrl_MotionEngine_ServiceSafety();
    assert(Ctrl_MotionEngine_HasFault());

    /* 不同轴待确认事件各自到期；不能因清M1而提前恢复仍待M3确认的STEP。 */
    Reset();Ctrl_MotionEngine_EnableLimitMonitoring(true);
    MotionFrame_t frame={20,0,0,20};assert(Ctrl_MotionEngine_PushFrame(&frame));
    Ctrl_MotionEngine_NotifyLimitSwitch(MOTION_LIMIT_M1);
    FakeRobot_AdvanceUs(9000);Ctrl_MotionEngine_NotifyLimitSwitch(MOTION_LIMIT_M3);
    FakeRobot_AdvanceUs(1000);Ctrl_MotionEngine_ServiceSafety();Tick(1);
    assert(fake_steps[0]==0);
    FakeRobot_AdvanceUs(9000);Ctrl_MotionEngine_ServiceSafety();Tick(1);
    assert(fake_steps[0]==1 && !Ctrl_MotionEngine_HasFault());

    /* 独立窗口跨32位毫秒回绕仍按无符号差值确认。 */
    Reset();FakeRobot_SetTimeUs((uint64_t)(UINT32_MAX-5U)*1000U);
    Ctrl_MotionEngine_EnableLimitMonitoring(true);fake_limits[1]=true;
    Ctrl_MotionEngine_NotifyLimitSwitch(MOTION_LIMIT_M2);
    FakeRobot_AdvanceUs(9000);Ctrl_MotionEngine_NotifyLimitSwitch(MOTION_LIMIT_M3);
    FakeRobot_AdvanceUs(1000);Ctrl_MotionEngine_ServiceSafety();
    assert(Ctrl_MotionEngine_HasFault());
}
int main(void)
{
    TestCountsAndWaitFrames(); TestInvalidAndQueueWrap(); TestLimitPauseAndFault();
    TestIndependentAxisDebounce();
    puts("motion_engine_tests: passed"); return 0;
}
