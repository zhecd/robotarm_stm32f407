/**
 * @file    app_calibration.h
 * @brief   Encoder zero-point calibration / coordinate alignment / 缂栫爜鍣ㄩ浂鐐规爣�?/ 鍧愭爣瀵归�?
 * @ingroup app
 *
 * Synchronizes the physical encoder zero position with the theoretical
 * step count after homing. Disables PID axes whose encoders fail to
 * calibrate.
 * 鍥為浂鍚庡皢鐗╃悊缂栫爜鍣ㄩ浂鐐逛笌鐞嗚姝ユ暟鍚屾�? 鏍囧畾澶辫触鐨勭紪鐮佸櫒浼氱鐢ㄥ�?PID 杞淬�?
 */

#ifndef __APP_CALIBRATION_H__
#define __APP_CALIBRATION_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool App_Calibration_Execute(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CALIBRATION_H__ */
