/* TMC2209单元测试替换HAL寄存器和阻塞UART端口，不替换真实驱动源码。 */
#ifndef TEST_TMC_MAIN_H
#define TEST_TMC_MAIN_H
#include <stdint.h>
typedef struct { uint32_t CR1,DR; } USART_TypeDef;
typedef struct { USART_TypeDef *Instance; } UART_HandleTypeDef;
typedef enum { HAL_OK=0,HAL_ERROR=1,HAL_BUSY=2,HAL_TIMEOUT=3 } HAL_StatusTypeDef;
#define USART_CR1_RE (1U<<2)
#define CLEAR_BIT(value,mask) ((value)&=~(mask))
#define SET_BIT(value,mask) ((value)|=(mask))
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)(uart))
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);
void HAL_Delay(uint32_t ms);
#endif
