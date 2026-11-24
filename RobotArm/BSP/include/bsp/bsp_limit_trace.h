/**
 * @file bsp_limit_trace.h
 * @brief 限位共享中断入口记录器：固定RAM保存配置/电平/pending，不改变触发与清除。
 */
#ifndef ROBOTARM_BSP_LIMIT_TRACE_H
#define ROBOTARM_BSP_LIMIT_TRACE_H
#include <stdbool.h>
#include <stdint.h>
#define BSP_LIMIT_TRACE_CAPACITY 64U
typedef struct {
    uint32_t tick_ms, cycles, pending, interrupt_mask, gpio_c_input;
    uint32_t exticr4, rising, falling, software_trigger, active_vector;
} BspLimitTraceRecord_t;
/** 只在EXTI15_10入口调用；HAL清pending之前记录，满后冻结，不循环覆盖。 */
void BSP_LimitTrace_CaptureVector(void);
/** 已发布记录不再修改；读取只用于主循环/电脑诊断，不暂停或写配置。 */
bool BSP_LimitTrace_ReadRecord(uint32_t index,BspLimitTraceRecord_t *out);
uint32_t BSP_LimitTrace_GetCount(void);
uint32_t BSP_LimitTrace_GetDropped(void);
#endif
