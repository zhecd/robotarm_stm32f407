#ifndef BSP_UART1_H
#define BSP_UART1_H

#include "usart.h"
#include <stdbool.h>
#include <stdint.h>

#define UART1_RX_BUF_SIZE 256U

void BSP_UART1_Init(void);
bool BSP_UART1_ReadLine(char *out_line, uint16_t max_len);
void BSP_UART1_RxCallback(void);
void BSP_UART1_SendString(const char *str);

#endif /* BSP_UART1_H */
