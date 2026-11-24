/** @file irq_evidence.h
 * @brief 只读审计的有限中断取证，不改限位电平、清除策略或安全预算。
 * @details 一个共享向量入口对应一条完整记录；共享向量退出时才发布。
 */
#ifndef AUDIT_IRQ_EVIDENCE_H
#define AUDIT_IRQ_EVIDENCE_H
#include <stdint.h>
#define IRQ_EVIDENCE_CAPACITY 64U
#define IRQ_AXIS_HEAD_CAPACITY 32U
#define IRQ_AXIS_TAIL_CAPACITY 32U
#define IRQ_AXIS_CAPACITY (IRQ_AXIS_HEAD_CAPACITY+IRQ_AXIS_TAIL_CAPACITY)
#define IRQ_AXIS_MAGIC 0x49524131U
typedef struct {
    uint32_t tick_ms,cycles_enter,pending_enter,mask_enter,gpio_c;
    uint32_t exticr3,exticr4,rising,falling,software_trigger,vector;
    uint32_t before[3],after[3],callback_mask,cycles_exit;
} IrqEvidenceRecord;
typedef struct {
    uint32_t magic,capacity,record_bytes,complete_count,vectors_total,dropped;
    uint32_t callbacks[3];
} IrqEvidenceHeader;
extern volatile IrqEvidenceHeader g_irq_evidence;
extern volatile IrqEvidenceRecord g_irq_records[IRQ_EVIDENCE_CAPACITY];

/** 每轴只选入该轴实际回调所在的完整向量；同一向量可进入多个轴。
 * vector_sequence用于跨轴关联，axis_sequence为该轴相关向量累计编号。
 * 不把一条共享向量拆成不存在的独立硬件事件。 */
typedef struct {
    uint32_t vector_sequence,axis_sequence;
    IrqEvidenceRecord record;
} IrqAxisRecord;
/** 32条首次历史永不覆盖，另32条为尾部环形区，tail_next指向下一写入槽。
 * 该轴在真实处理返回后已被屏蔽则冻结，不由记录器屏蔽/解锁IRQ。
 * SWD读取header→records→header，两头一致且revision为偶数才是稳定快照。
 * revision是自然回绕的序列锁，统计字段则饱和；冻结后仍记录异常的后续调用数。 */
typedef struct {
    uint32_t magic,axis,head_capacity,tail_capacity,record_bytes;
    uint32_t head_count,tail_count,tail_next,events_total,tail_overwrites;
    uint32_t frozen,freeze_imr,freeze_vector,ignored_after_freeze,revision;
} IrqAxisHeader;
extern volatile IrqAxisHeader g_irq_axis_evidence[3];
extern volatile IrqAxisRecord g_irq_axis_records[3][IRQ_AXIS_CAPACITY];
#endif
