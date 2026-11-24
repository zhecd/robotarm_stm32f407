/** UART/DMA 的最小 HAL 替身，仅在 bsp_uart_tests 目标的私有包含目录生效。 */
#ifndef ROBOTARM_TEST_USART_H
#define ROBOTARM_TEST_USART_H
#include <stdint.h>
typedef struct { uint32_t counter; } DMA_HandleTypeDef;
typedef struct { void *Instance; DMA_HandleTypeDef *hdmarx; } UART_HandleTypeDef;
typedef enum { HAL_OK=0, HAL_ERROR=1 } HAL_StatusTypeDef;
extern UART_HandleTypeDef huart1;
#define USART1 ((void *)1)
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->counter)
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)(uart))
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
#endif
