/**
 * @file error_code.h
 * @brief 统一错误码：0 成功，正值为已接收但尚未完成，负值为拒绝或失败。
 */

#ifndef __ERROR_CODE_H__
#define __ERROR_CODE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Standardized error codes. ERR_OK = 0 for truthiness. */
typedef enum {
    /* Command accepted; its asynchronous validation is still in progress. */
    ERR_PENDING         =  1,
    /* Command accepted into the waypoint FIFO; execution follows FIFO order. */
    ERR_QUEUED          =  2,
    ERR_OK              =  0,    /* Success */
    ERR_NULL_PARAM      = -1,    /* Null pointer parameter */
    ERR_I2C_FAIL        = -2,    /* I2C communication failure */
    ERR_UART_FAIL       = -3,    /* UART communication failure */
    ERR_TIMEOUT         = -4,    /* 操作超时。 */
    ERR_BUFFER_FULL     = -5,    /* 缓冲区已满。 */
    ERR_NOT_INITIALIZED = -6,    /* Module not initialized */
    ERR_OUT_OF_RANGE    = -7,    /* 数值超出有效范围。 */
    ERR_BUSY            = -8,    /* 当前状态不允许接收该请求。 */
    ERR_ENCODER_FAIL    = -9,    /* Encoder read failure */
    ERR_SOFT_LIMIT      = -10,   /* Actual joint angle exceeded software limit. */
} ErrorCode_t;

#ifdef __cplusplus
}
#endif

#endif /* __ERROR_CODE_H__ */
