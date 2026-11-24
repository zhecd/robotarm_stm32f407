/** 最小对照统计真实源码：计数解释、独立预算及无符号时差；不模拟W1C硬件。 */
#include "probe_logic.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    ProbeStats_t s;
    ProbeLogic_Init(&s,4U);ProbeLogic_Sample(&s,4U);
    assert(s.samples==1 && s.low_samples[2]==1 && s.falls[2]==0);
    ProbeLogic_Sample(&s,0U);ProbeLogic_Sample(&s,1U);ProbeLogic_Sample(&s,5U);
    assert(s.rises[2]==1 && s.falls[0]==1 && s.falls[2]==1);
    assert(!s.pending[0] && !s.irq_vectors);
    ProbeLogic_Init(&s,0U);
    for(unsigned i=0;i<1000;i++)assert(ProbeLogic_Pending(&s,4,5,0,0)==0);
    assert(s.pending[2]==1000 && s.after_ack[2]==1000 && s.after_ack[0]==0);
    assert(!s.irq_vectors && !s.masked); /* 轮询观测不能冒充IRQ次数。 */
    ProbeLogic_Init(&s,0U);
    for(unsigned i=0;i<255;i++)assert(ProbeLogic_Pending(&s,4,0,1,1)==0);
    assert(ProbeLogic_Pending(&s,4,0,1,1)==4 && s.pending[2]==256);
    assert(ProbeLogic_Pending(&s,1,0,1,1)==4 && s.guard_count[0]==1);
    ProbeLogic_Init(&s,0U);
    for(unsigned i=0;i<255;i++)ProbeLogic_Pending(&s,2,0,UINT32_MAX-5U,1);
    assert(ProbeLogic_Pending(&s,2,0,5,1)==0 && s.guard_count[1]==1);
    ProbeLogic_Pending(&s,0,7,5,1);
    assert(s.spurious_vectors==1 && s.pending[0]==0 && s.after_ack[1]==0);
    assert(sizeof(ProbeRecord_t)==52U);
    puts("limit_probe_logic_tests: passed");return 0;
}
