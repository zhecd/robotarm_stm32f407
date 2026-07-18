/**
 * @file    bsp_tmc2209.c
 * @brief   TMC2209 UART register configuration implementation / TMC2209 UART 瀵勫瓨鍣ㄩ厤缃疄鐜?
 * @ingroup bsp
 */

#include "driver/drv_tmc2209.h"
#include "bsp_driver_config.h"
#include <stdio.h>

#define TMC2209_REG_GCONF             0x00U
#define TMC2209_REG_IHOLD_IRUN        0x10U
#define TMC2209_REG_CHOPCONF          0x6CU
#define TMC2209_WRITE_SYNC            0x05U
#define TMC2209_UART_WRITE_FLAG       0x80U
#define TMC2209_GCONF_MSTEP_REG_SELECT (1UL << 7)
#define TMC2209_GCONF_MULTISTEP_FILT  (1UL << 6)
#define TMC2209_GCONF_I_SCALE_ANALOG  (1UL << 0)
#define TMC2209_CHOPCONF_BASE         0x10000053UL
#define TMC2209_CHOPCONF_INTPOL       (1UL << 28)
#define TMC2209_CHOPCONF_MRES_SHIFT   24U
#define TMC2209_IHOLDDELAY_SHIFT      16U
#define TMC2209_IRUN_SHIFT            8U
#define TMC2209_CURRENT_MASK          0x1FU
#define TMC2209_WRITE_PACKET_SIZE     8U
#define TMC2209_UART_TX_TIMEOUT_MS    100U
#define TMC2209_WRITE_SETTLE_MS       5U

/* 鈹€鈹€ CRC8 calculation / CRC8 璁＄�?鈹€鈹€ */

static uint8_t CalcCRC8(const uint8_t *datagram, uint8_t len)
{
    uint8_t crc = 0U;
    for (uint8_t i = 0U; i < len; i++) {
        uint8_t byte = datagram[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((uint8_t)((crc >> 7) ^ (byte & 0x01U)) != 0U)
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            else
                crc <<= 1;
            byte >>= 1;
        }
    }
    return crc;
}

/* 鈹€鈹€ Microstep to MRES lookup / 寰鏁拌浆 MRES 鏌ユ�?鈹€鈹€ */

static uint32_t MicrostepsToMres(uint16_t microsteps)
{
    switch (microsteps) {
    case 256U: return 0U;
    case 128U: return 1U;
    case 64U:  return 2U;
    case 32U:  return 3U;
    case 16U:  return 4U;
    case 8U:   return 5U;
    case 4U:   return 6U;
    case 2U:   return 7U;
    case 1U:   return 8U;
    default:   return 4U;
    }
}

/* 鈹€鈹€ UART register write with CRC8 / �?CRC8 �?UART 瀵勫瓨鍣ㄥ啓�?鈹€鈹€ */

static void WriteReg(UART_HandleTypeDef *huart, uint8_t addr, uint8_t reg, uint32_t data)
{
    uint8_t packet[TMC2209_WRITE_PACKET_SIZE];
    packet[0] = TMC2209_WRITE_SYNC;
    packet[1] = addr;
    packet[2] = (uint8_t)(reg | TMC2209_UART_WRITE_FLAG);
    packet[3] = (uint8_t)((data >> 24) & 0xFFU);
    packet[4] = (uint8_t)((data >> 16) & 0xFFU);
    packet[5] = (uint8_t)((data >>  8) & 0xFFU);
    packet[6] = (uint8_t)( data        & 0xFFU);
    packet[7] = CalcCRC8(packet, 7U);

    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, packet, TMC2209_WRITE_PACKET_SIZE, TMC2209_UART_TX_TIMEOUT_MS);
    if (status != HAL_OK) {
        printf("# [TMC2209] TX fail: addr=%d reg=0x%02X err=%d\r\n", addr, reg, status);
    }
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)huart->Instance->DR;
    HAL_Delay(TMC2209_WRITE_SETTLE_MS);
}

/* 鈹€鈹€ Public API / 鍏紑鎺ュ彛 鈹€鈹€ */

void Drv_TMC2209_ConfigNode(UART_HandleTypeDef *huart, uint8_t node_addr,
                            uint16_t microsteps, uint8_t irun, uint8_t ihold)
{
    /* GCONF: StealthChop settings / StealthChop 璁剧�?*/
    uint32_t gconf = TMC2209_GCONF_MSTEP_REG_SELECT |
                     TMC2209_GCONF_MULTISTEP_FILT  |
                     TMC2209_GCONF_I_SCALE_ANALOG;
    WriteReg(huart, node_addr, TMC2209_REG_GCONF, gconf);

    /* CHOPCONF: microstep + interpolation / 寰�?+ 鎻掑�?*/
    uint32_t mres = MicrostepsToMres(microsteps);
    uint32_t chopconf = TMC2209_CHOPCONF_BASE |
                        TMC2209_CHOPCONF_INTPOL |
                        (mres << TMC2209_CHOPCONF_MRES_SHIFT);
    WriteReg(huart, node_addr, TMC2209_REG_CHOPCONF, chopconf);

    /* IHOLD_IRUN: current settings / 鐢垫祦璁剧疆 */
    uint32_t ihold_irun = (6UL << TMC2209_IHOLDDELAY_SHIFT) |
                          (((uint32_t)irun & TMC2209_CURRENT_MASK) << TMC2209_IRUN_SHIFT) |
                          ((uint32_t)ihold & TMC2209_CURRENT_MASK);
    WriteReg(huart, node_addr, TMC2209_REG_IHOLD_IRUN, ihold_irun);
}

void Drv_TMC2209_InitBus(UART_HandleTypeDef *huart)
{
    for (uint8_t addr = 0U; addr <= 3U; addr++) {
        Drv_TMC2209_ConfigNode(huart, addr, 16U, TMC_IRUN_HIGH, TMC_IHOLD_STRONG);
    }
}
