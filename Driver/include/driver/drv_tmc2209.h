/**
 * @file    bsp_tmc2209.h
 * @brief   TMC2209 stepper driver UART configuration interface / TMC2209 姝ヨ繘椹卞姩 UART 閰嶇疆鎺ュ彛
 * @ingroup bsp
 *
 * Communicates via USART6 half-duplex single-wire to up to 4 nodes / 閫氳�?USART6 鍗婂弻宸ュ崟绾胯繛鎺ユ渶�?4 涓妭鐐?
 * Configures GCONF, CHOPCONF, and IHOLD_IRUN registers / 閰嶇�?GCONF/CHOPCONF/IHOLD_IRUN 瀵勫瓨鍣?
 */

#ifndef __Drv_TMC2209_H__
#define __Drv_TMC2209_H__

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize all 4 TMC2209 nodes on the bus / 鍒濆鍖栨€荤嚎涓婂叏閮?4 涓妭鐐?*/
void Drv_TMC2209_InitBus(UART_HandleTypeDef *huart);

/** Configure a single TMC2209 node / 閰嶇疆鍗曚釜 TMC2209 鑺傜�?*/
void Drv_TMC2209_ConfigNode(UART_HandleTypeDef *huart, uint8_t node_addr,
                            uint16_t microsteps, uint8_t irun, uint8_t ihold);

#ifdef __cplusplus
}
#endif

#endif /* __Drv_TMC2209_H__ */
