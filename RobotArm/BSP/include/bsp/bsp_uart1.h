/**
 * @file    bsp_uart1.h
 * @brief   UART1 ring-buffer line reader with printf retarget / UART1 鐜舰缂撳啿琛岃鍙?+ printf 閲嶅畾鍚?
 * @ingroup bsp
 *
 * Uses interrupt-driven RX with a 256-byte ring buffer / 涓柇椹卞姩鎺ユ�? 256瀛楄妭鐜舰缂撳�?
 * Supports line-delimited reads with timeout fallback / 鏀寔鎹㈣绗﹀垎闅?+ 瓒呮椂鍥為€�?
 * Redirects printf() output via __io_putchar / 閫氳�?__io_putchar 閲嶅畾鍚?printf
 */

#ifndef __BSP_UART1_H__
#define __BSP_UART1_H__

#include "usart.h"
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

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART1_H__ */
