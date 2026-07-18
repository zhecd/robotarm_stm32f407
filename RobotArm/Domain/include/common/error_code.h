/**
 * @file    error_code.h
 * @brief   Unified error code definitions / 缁熶竴閿欒鐮佸畾涔?
 * @ingroup common
 *
 * All public APIs return ErrorCode_t to enable consistent error propagation
 * across BSP, control, and application layers.
 * 鎵€鏈夊叕寮�?API 杩斿�?ErrorCode_t锛屽疄鐜拌法 BSP/鎺у埗/搴旂敤灞傜殑涓€鑷撮敊璇紶閫掋�?
 */

#ifndef __ERROR_CODE_H__
#define __ERROR_CODE_H__

#ifdef __cplusplus
extern "C" {
#endif

/** Standardized error codes. ERR_OK = 0 for truthiness. / 鏍囧噯鍖栭敊璇�? ERR_OK=0 渚夸簬鐪熷€煎垽�?*/
typedef enum {
    /* Command accepted; its asynchronous validation is still in progress. */
    ERR_PENDING         =  1,
    /* Command accepted into the waypoint FIFO; execution follows FIFO order. */
    ERR_QUEUED          =  2,
    ERR_OK              =  0,    /* Success / 鎴愬�?*/
    ERR_NULL_PARAM      = -1,    /* Null pointer parameter / 绌烘寚閽堝弬�?*/
    ERR_I2C_FAIL        = -2,    /* I2C communication failure / I2C 閫氫俊澶辫触 */
    ERR_UART_FAIL       = -3,    /* UART communication failure / UART 閫氫俊澶辫触 */
    ERR_TIMEOUT         = -4,    /* Operation timeout / 鎿嶄綔瓒呮椂 */
    ERR_BUFFER_FULL     = -5,    /* Buffer full / 缂撳啿鍖烘弧 */
    ERR_NOT_INITIALIZED = -6,    /* Module not initialized / 妯″潡鏈垵濮嬪寲 */
    ERR_OUT_OF_RANGE    = -7,    /* Value out of valid range / 鏁板€艰秴鍑烘湁鏁堣寖鍥?*/
    ERR_BUSY            = -8,    /* Resource busy / 璧勬簮蹇?*/
    ERR_ENCODER_FAIL    = -9,    /* Encoder read failure / 缂栫爜鍣ㄨ鍙栧け璐?*/
    ERR_SOFT_LIMIT      = -10,   /* Actual joint angle exceeded software limit. */
} ErrorCode_t;

#ifdef __cplusplus
}
#endif

#endif /* __ERROR_CODE_H__ */
