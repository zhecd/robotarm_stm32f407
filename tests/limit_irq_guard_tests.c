/** 真实过载锁存服务：冻结时基、各轴隔离、窗口边界和毫秒回绕。 */
#include "limit_irq_guard.h"
#include "fake_robot.h"
#include <stdio.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);return 1; } } while (0)
int main(void)
{
    FakeRobot_Reset();
    CHECK(!LimitIrqGuard_OnEdgeFromISR(255));
    for (unsigned n=1;n<LIMIT_IRQ_GUARD_BURST_LIMIT;n++) CHECK(!LimitIrqGuard_OnEdgeFromISR(0));
    CHECK(LimitIrqGuard_GetLatchMask()==0);
    CHECK(LimitIrqGuard_OnEdgeFromISR(0)); CHECK(LimitIrqGuard_GetLatchMask()==1);
    for (unsigned n=0;n<10000;n++) CHECK(LimitIrqGuard_OnEdgeFromISR(0));
    for (unsigned n=1;n<256;n++) CHECK(!LimitIrqGuard_OnEdgeFromISR(1));
    FakeRobot_AdvanceUs(10000); /* 窗口边界重置，旧批次不能并入新窗口。 */
    CHECK(!LimitIrqGuard_OnEdgeFromISR(1));
    for (unsigned n=1;n<255;n++) CHECK(!LimitIrqGuard_OnEdgeFromISR(1));
    CHECK(LimitIrqGuard_OnEdgeFromISR(1)); CHECK(LimitIrqGuard_GetLatchMask()==3);
    FakeRobot_SetTimeUs(((uint64_t)UINT32_MAX-5U)*1000U);
    for (unsigned n=1;n<256;n++) CHECK(!LimitIrqGuard_OnEdgeFromISR(2));
    FakeRobot_AdvanceUs(9000); CHECK(LimitIrqGuard_OnEdgeFromISR(2));
    CHECK(LimitIrqGuard_GetLatchMask()==7);
    FakeRobot_AdvanceUs(1000000); CHECK(LimitIrqGuard_GetLatchMask()==7);
    puts("limit_irq_guard_tests: passed");return 0;
}
