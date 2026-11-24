/**
 * @file bsp_i2c_recovery.h
 * @brief 单主机编码器I2C故障恢复端口；只操作已绑定总线，不访问电机接口。
 */
#ifndef ROBOTARM_BSP_I2C_RECOVERY_H
#define ROBOTARM_BSP_I2C_RECOVERY_H
#include "stm32f4xx_hal.h"
#include <stdbool.h>
/** 主循环中、HAL读操作已经失败且句柄解锁后调用。
 * 最多9个SCL恢复时钟、每次等待SCL释放最多1ms，随后产生STOP并恢复I2C复用。
 * 未识别总线或正在传输时拒绝；失败也尝试恢复外设配置，绝不无限等待。
 */
bool BSP_I2C_RecoverBus(I2C_HandleTypeDef *bus);
#endif
