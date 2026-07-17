/**
 * @file    error_code.h
 * @brief   Unified error code definitions / 统一错误码定义
 * @ingroup common
 *
 * All public APIs return ErrorCode_t to enable consistent error propagation
 * across BSP, control, and application layers.
 * 所有公开 API 返回 ErrorCode_t，实现跨 BSP/控制/应用层的一致错误传递。
 */

#ifndef __ERROR_CODE_H__
#define __ERROR_CODE_H__

#ifdef __cplusplus
extern "C" {
#endif

/** Standardized error codes. ERR_OK = 0 for truthiness. / 标准化错误码, ERR_OK=0 便于真值判断 */
typedef enum {
    ERR_OK              =  0,    /* Success / 成功 */
    ERR_NULL_PARAM      = -1,    /* Null pointer parameter / 空指针参数 */
    ERR_I2C_FAIL        = -2,    /* I2C communication failure / I2C 通信失败 */
    ERR_UART_FAIL       = -3,    /* UART communication failure / UART 通信失败 */
    ERR_TIMEOUT         = -4,    /* Operation timeout / 操作超时 */
    ERR_BUFFER_FULL     = -5,    /* Buffer full / 缓冲区满 */
    ERR_NOT_INITIALIZED = -6,    /* Module not initialized / 模块未初始化 */
    ERR_OUT_OF_RANGE    = -7,    /* Value out of valid range / 数值超出有效范围 */
    ERR_BUSY            = -8,    /* Resource busy / 资源忙 */
    ERR_ENCODER_FAIL    = -9,    /* Encoder read failure / 编码器读取失败 */
    ERR_SOFT_LIMIT      = -10,   /* Actual joint angle exceeded software limit. */
} ErrorCode_t;

#ifdef __cplusplus
}
#endif

#endif /* __ERROR_CODE_H__ */
