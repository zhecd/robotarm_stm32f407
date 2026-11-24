/** @file contact_gate.h
 * @brief 仅接管当前寻限位轴的有效低电平；先停输出，再屏蔽该触点边沿。
 * GPIO仍逐毫秒采样；其他轴和高电平毛刺仍走原IRQ预算。已有故障不被清除。
 * 当前专用试验不退回；被接管触点保留屏蔽至试验结束，不恢复运动许可。
 */
#ifndef SEEK_CONTACT_GATE_H
#define SEEK_CONTACT_GATE_H
#include "seek_logic.h"
typedef struct { volatile Seek *seek; uint8_t masked_mask; } SeekContact;
/** 调用方持有短临界区，串行化SysTick与EXTI；不允许打印或阻塞。 */
bool SeekContact_OnEdge(SeekContact *gate, bool active, uint8_t axis,
                       uint8_t limits, uint8_t irq_latch, uint32_t now);
void SeekContactPort_StopOutputs(void);
void SeekContactPort_MaskAxis(uint8_t axis);
#endif
