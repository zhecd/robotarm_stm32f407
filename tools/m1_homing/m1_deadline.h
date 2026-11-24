/** @file m1_deadline.h
 * @brief TIM5截止端口；装载更新事件不能被误判为真正超时。
 */
#ifndef M1_DEADLINE_H
#define M1_DEADLINE_H
#include <stdbool.h>
#include <stdint.h>
/** 调用者须先禁用执行器并进入短临界区。只装载，不启动计数。
 * 时钟固定84MHz，分频后10kHz；期限仅允许1..5500ms，非法参数也停止定时器。
 */
bool M1Deadline_Prepare(uint32_t deadline_ms);
/** 仅由TIM5 ISR调用；真实UIF且UIE启用才返回true并停止计数。
 * 调用者在true时立即禁用输出并锁存截止故障；无UIF的残余IRQ不能当作截止。
 */
bool M1Deadline_ExpiredFromISR(void);
#endif
