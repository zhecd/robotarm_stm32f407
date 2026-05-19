/**
 * @file    bsp_tmc2209.h
 * @brief   TMC2209 stepper driver UART configuration interface / TMC2209 步进驱动 UART 配置接口
 * @ingroup bsp
 *
 * Communicates via USART6 half-duplex single-wire to up to 4 nodes / 通过 USART6 半双工单线连接最多 4 个节点
 * Configures GCONF, CHOPCONF, and IHOLD_IRUN registers / 配置 GCONF/CHOPCONF/IHOLD_IRUN 寄存器
 */

#ifndef __BSP_TMC2209_H__
#define __BSP_TMC2209_H__

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize all 4 TMC2209 nodes on the bus / 初始化总线上全部 4 个节点 */
void BSP_TMC2209_InitBus(UART_HandleTypeDef *huart);

/** Configure a single TMC2209 node / 配置单个 TMC2209 节点 */
void BSP_TMC2209_ConfigNode(UART_HandleTypeDef *huart, uint8_t node_addr,
                            uint16_t microsteps, uint8_t irun, uint8_t ihold);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TMC2209_H__ */
