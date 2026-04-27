#ifndef BSP_TMC2209_H
#define BSP_TMC2209_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define TMC_IRUN_HIGH    24U
#define TMC_IRUN_MED     16U
#define TMC_IRUN_LOW     8U

#define TMC_IHOLD_STRONG 12U
#define TMC_IHOLD_COOL   4U

void BSP_TMC2209_InitBus(UART_HandleTypeDef *huart);
void BSP_TMC2209_ConfigNode(UART_HandleTypeDef *huart, uint8_t node_addr, uint16_t microsteps, uint8_t irun, uint8_t ihold);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TMC2209_H */
