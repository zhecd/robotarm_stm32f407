/** @file m1_rx.h
 * @brief 独立试验的有界串口接收缓存；ISR收字节，主循环取字节。
 * 主循环调用时须屏蔽USART1中断或使用短临界区；本模块不解析、不执行指令。
 */
#ifndef M1_RX_H
#define M1_RX_H
#include <stdbool.h>
#include <stdint.h>
#define M1_RX_CAPACITY 128U
typedef struct {
    uint8_t bytes[M1_RX_CAPACITY];
    uint16_t head,tail;
    bool fault;
    uint32_t uart_errors,overflows;
} M1Rx;
/** UART错误/缓存满必须立即由调用者禁用输出；故障后不接收后续字节。 */
void M1Rx_Error(volatile M1Rx *r);
bool M1Rx_Push(volatile M1Rx *r,uint8_t ch);
bool M1Rx_Pop(volatile M1Rx *r,uint8_t *ch);
/** 丢弃故障前后缓存，返回故障；解析器随后丢弃至下一个行尾再同步。 */
bool M1Rx_TakeFault(volatile M1Rx *r);
/** 动作配置之后仅允许遗留CR/LF；必须在临界区使用，其他输入拒绝启动。 */
bool M1Rx_DrainTail(volatile M1Rx *r);
#endif
