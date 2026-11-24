#include "limit_observation_service.h"
#include "fake_robot.h"
#include <assert.h>
#include <stdio.h>
static LimitObservationStatus_t Status(void)
{
    LimitObservationStatus_t value;
    LimitObservation_GetStatus(&value);return value;
}
int main(void)
{
    FakeRobot_Reset();LimitObservation_Init(0,0);
    LimitObservation_Sample(0,9);assert(Status().valid_mask==0);
    LimitObservation_Sample(0,10);assert(Status().valid_mask==7);
    /* 复现短暂边沿：原始计数增加，但电平不足10ms不算一次有效按下。 */
    LimitObservation_OnEdgeFromISR(2);LimitObservation_Sample(4,11);
    LimitObservation_Sample(0,14);LimitObservation_Sample(0,24);
    assert(Status().edges[2]==1 && Status().presses[2]==0 && Status().stable_mask==0);
    for(unsigned axis=0;axis<3;axis++) {
        unsigned base=100+axis*100;
        LimitObservation_OnEdgeFromISR((uint8_t)axis);
        LimitObservation_Sample((uint8_t)(1U<<axis),base);
        LimitObservation_Sample((uint8_t)(1U<<axis),base+9);
        assert(Status().presses[axis]==0);
        LimitObservation_Sample((uint8_t)(1U<<axis),base+10);
        assert(Status().presses[axis]==1 && Status().stable_mask==(1U<<axis));
        LimitObservation_Sample((uint8_t)(1U<<axis),base+30);
        assert(Status().presses[axis]==1); /* 持续按住只计一次。 */
        LimitObservation_Sample(0,base+31);LimitObservation_Sample(0,base+41);
        assert(Status().stable_mask==0);
    }
    LimitObservation_OnEdgeFromISR(255);LimitObservation_GetStatus(NULL);
    assert(Status().edges[0]==1 && Status().edges[1]==1 && Status().edges[2]==2);
    /* 启动时已按住也需资格时间；毫秒回绕仍按差值确认。 */
    LimitObservation_Init(255,UINT32_MAX-5U);
    assert(Status().edges[2]==0 && Status().valid_mask==0);
    LimitObservation_Sample(7,3);assert(Status().valid_mask==0);
    LimitObservation_Sample(7,4);
    assert(Status().stable_mask==7 && Status().valid_mask==7);
    assert(Status().presses[0]==1 && Status().presses[1]==1 && Status().presses[2]==1);
    /* 时基完全不前进也须限流；锁存独立于电平消抖，不会把限位改为释放。 */
    FakeRobot_Reset();LimitObservation_Init(2,0);
    for(unsigned i=1;i<LIMIT_OBSERVATION_IRQ_BURST_LIMIT;i++)
        assert(!LimitObservation_OnEdgeFromISR(1));
    assert(LimitObservation_OnEdgeFromISR(1));
    assert(Status().edges[1]==256 && Status().irq_latch_mask==2);
    for(unsigned i=0;i<1000;i++) assert(LimitObservation_OnEdgeFromISR(1));
    assert(Status().edges[1]==256); /* 不把屏蔽后冻结计数误报为完整边沿统计。 */
    LimitObservation_Sample(2,10);
    assert(Status().stable_mask==2 && Status().presses[1]==1);
    LimitObservation_Sample(0,11);LimitObservation_Sample(0,21);
    assert(Status().stable_mask==0 && Status().irq_latch_mask==2);
    assert(!LimitObservation_OnEdgeFromISR(0) && Status().edges[0]==1);
    assert(!LimitObservation_OnEdgeFromISR(255));
    /* 正常分散的回调不因累计超过256次而误锁存。时间回绕也要正确换窗。 */
    FakeRobot_Reset();LimitObservation_Init(0,0);
    for(unsigned i=0;i<600;i++) {
        FakeRobot_AdvanceUs(10000);
        assert(!LimitObservation_OnEdgeFromISR(0));
    }
    assert(Status().edges[0]==600 && Status().irq_latch_mask==0);
    FakeRobot_Reset();FakeRobot_SetTimeUs((uint64_t)(UINT32_MAX-5U)*1000U);
    LimitObservation_Init(0,UINT32_MAX-5U);
    for(unsigned i=0;i<255;i++) assert(!LimitObservation_OnEdgeFromISR(2));
    FakeRobot_AdvanceUs(10000);assert(!LimitObservation_OnEdgeFromISR(2));
    assert(Status().edges[2]==256 && Status().irq_latch_mask==0);
    LimitObservation_Init(0,4);assert(Status().irq_latch_mask==0 && Status().edges[2]==0);
    assert(fake_enable_true_count==0 && fake_wave_rises[0]==0);
    puts("limit_observation_tests: passed");return 0;
}
