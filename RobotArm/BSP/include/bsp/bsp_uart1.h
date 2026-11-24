/**
 * @file bsp_uart1.h
 * @brief UART1 传输：DMA 环形接收、中断发送、错误恢复与文本组帧适配。
 */

#ifndef __BSP_UART1_H__
#define __BSP_UART1_H__


#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_UART1_Init(void);
bool BSP_UART1_ReadLine(char *line, uint16_t max_len);
bool BSP_UART1_TakeLineTimeout(void);
bool BSP_UART1_TakeRxOverflow(void);
bool BSP_UART1_TakeTxOverflow(void);
/* Wait for queued diagnostic output before a controlled fatal stop. */
bool BSP_UART1_FlushTx(uint32_t timeout_ms);
void BSP_UART1_DiscardRx(void);
void BSP_UART1_RxCallback(void);
void BSP_UART1_SendString(const char *str);

/* 首次RX异常快照：reason1=DMA覆盖、2=UART错误回调、3=超长行。
 * 计数/位置单位字节；BSP_Init清除，普通DiscardRx不抹去首次故障证据。 */
typedef struct {
    uint32_t reason,now_ms,wraps,observed_wraps,dma_pos,head,tail,produced,unread;
} BspUartRxFault_t;
void BSP_UART1_GetRxFault(BspUartRxFault_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART1_H__ */
