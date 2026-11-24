/**
 * @file probe_logic.h
 * @brief 最小限位对照的纯统计合同；不操作寄存器，不代表运动保护实现。
 */
#ifndef LIMIT_PROBE_LOGIC_H
#define LIMIT_PROBE_LOGIC_H
#include <stdint.h>
#define PROBE_PHASE_COUNT 4U
#define PROBE_PHASE_MS 20000U
#define PROBE_TRACE_CAPACITY 128U
enum { PROBE_OFF, PROBE_POLL, PROBE_IRQ, PROBE_OFF_AFTER, PROBE_DONE };
typedef struct {
    uint32_t samples, low_samples[3], falls[3], rises[3];
    uint32_t pending[3], after_ack[3];
    uint32_t irq_vectors, spurious_vectors, masked;
    uint32_t guard_count[3], guard_window[3], last_low_mask;
} ProbeStats_t;
/** 输入低电平/待处理位图均为逻辑bit0..2，不接受GPIO位号。 */
void ProbeLogic_Init(volatile ProbeStats_t *s, uint32_t initial_low_mask);
void ProbeLogic_Sample(volatile ProbeStats_t *s, uint32_t low_mask);
/** IRQ时统计10ms/256次的每轴预算并返回要屏蔽的位图；轮询不做IRQ预算。
 * after仅统计本次handled位中读回仍pending的轴，不能当作清除失败的直接证明。
 */
uint32_t ProbeLogic_Pending(volatile ProbeStats_t *s, uint32_t handled,
                            uint32_t after, uint32_t now_ms, uint32_t from_irq);
typedef struct {
    uint32_t ms, phase, source, pending_before, handled, pending_after;
    uint32_t gpio_before, gpio_after, imr, falling, rising, exticr4, cycles;
} ProbeRecord_t;
typedef struct {
    uint32_t magic, version, ready, done, clock_hz, ms, phase, phase_start;
    uint32_t trace_count, trace_dropped, clock_error;
    ProbeStats_t stats[PROBE_PHASE_COUNT];
} ProbeState_t;
_Static_assert(sizeof(ProbeRecord_t)==52U,"Readback record layout");
#endif
