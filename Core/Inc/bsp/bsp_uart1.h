/**
 * @file    bsp_uart1.h
 * @brief   UART1 ring-buffer line reader with printf retarget / UART1 环形缓冲行读取 + printf 重定向
 * @ingroup bsp
 *
 * Uses interrupt-driven RX with a 256-byte ring buffer / 中断驱动接收, 256字节环形缓冲
 * Supports line-delimited reads with timeout fallback / 支持换行符分隔 + 超时回退
 * Redirects printf() output via __io_putchar / 通过 __io_putchar 重定向 printf
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
void BSP_UART1_DiscardRx(void);
void BSP_UART1_RxCallback(void);
void BSP_UART1_SendString(const char *str);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART1_H__ */
