/** 真实BSP快照：有限记录、先写后发布、满后冻结，不写清标志/引脚/配置。 */
#include "main.h"
#include "bsp/bsp_limit_trace.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
TestExti test_exti;
TestGpio test_gpio;
TestDwt test_dwt;
TestSyscfg test_syscfg;
TestScb test_scb;
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
int main(void)
{
    BspLimitTraceRecord_t out;
    assert(BSP_LimitTrace_GetCount()==0 && !BSP_LimitTrace_ReadRecord(0,&out));
    test_exti=(TestExti){0x1000,0x1c04,0,0x1c04,0};
    test_gpio.IDR=0x1c00;test_syscfg.EXTICR[3]=2;test_scb.ICSR=0x8000038;
    TestExti initial=test_exti;
    for(unsigned i=0;i<BSP_LIMIT_TRACE_CAPACITY;i++) {
        tick=i+100;test_dwt.CYCCNT=i*168U;
        BSP_LimitTrace_CaptureVector();
        assert(BSP_LimitTrace_GetCount()==i+1 && BSP_LimitTrace_ReadRecord(i,&out));
        assert(out.tick_ms==tick && out.cycles==i*168U && out.pending==0x1000);
        assert(out.gpio_c_input==0x1c00 && out.exticr4==2 && out.active_vector==56);
        assert(out.rising==0 && out.falling==0x1c04 && out.software_trigger==0);
        assert(out.interrupt_mask==0x1c04 && !memcmp(&test_exti,&initial,sizeof(initial)));
    }
    tick=999;test_exti.PR=0;
    BSP_LimitTrace_CaptureVector();BSP_LimitTrace_CaptureVector();
    assert(BSP_LimitTrace_GetCount()==64 && BSP_LimitTrace_GetDropped()==2);
    assert(BSP_LimitTrace_ReadRecord(0,&out) && out.tick_ms==100 && out.pending==0x1000);
    assert(!BSP_LimitTrace_ReadRecord(64,&out) && !BSP_LimitTrace_ReadRecord(0,NULL));
    puts("limit_vector_trace_tests: passed");return 0;
}
