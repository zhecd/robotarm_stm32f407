/**
 * @file bsp_limit_trace.c
 * @brief 波形诊断的入口取证：只读寄存器、固定长度RAM写入，无打印、延时或I2C。
 */
#include "bsp/bsp_limit_trace.h"
#include "main.h"
#include <stddef.h>
_Static_assert(sizeof(BspLimitTraceRecord_t)==40U,"Trace record layout must remain 10 uint32 words");
static volatile BspLimitTraceRecord_t s_limit_trace[BSP_LIMIT_TRACE_CAPACITY];
static volatile uint32_t s_limit_trace_count, s_limit_trace_dropped;
void BSP_LimitTrace_CaptureVector(void)
{
    uint32_t index=s_limit_trace_count;
    if (index>=BSP_LIMIT_TRACE_CAPACITY) {
        if (s_limit_trace_dropped!=UINT32_MAX) ++s_limit_trace_dropped;
        return;
    }
    /* 寄存器依次读取，外部信号仍可变化；这是入口快照，不是模拟波形或原子同时采样。 */
    BspLimitTraceRecord_t record;
    record.pending=EXTI->PR;
    record.interrupt_mask=EXTI->IMR;
    record.gpio_c_input=GPIOC->IDR;
    record.cycles=DWT->CYCCNT;
    record.tick_ms=HAL_GetTick();
    record.exticr4=SYSCFG->EXTICR[3];
    record.rising=EXTI->RTSR;
    record.falling=EXTI->FTSR;
    record.software_trigger=EXTI->SWIER;
    record.active_vector=SCB->ICSR&0x1FFU;
    s_limit_trace[index]=record;
    s_limit_trace_count=index+1U; /* 完整记录写入后发布；已发布槽不再覆盖。 */
}
bool BSP_LimitTrace_ReadRecord(uint32_t index,BspLimitTraceRecord_t *out)
{
    if (!out || index>=s_limit_trace_count || index>=BSP_LIMIT_TRACE_CAPACITY) return false;
    *out=s_limit_trace[index];return true;
}
uint32_t BSP_LimitTrace_GetCount(void) { return s_limit_trace_count; }
uint32_t BSP_LimitTrace_GetDropped(void) { return s_limit_trace_dropped; }
