/**
 * @file    app_calibration.h
 * @brief   Encoder zero-point calibration / coordinate alignment / 编码器零点标定 / 坐标对齐
 * @ingroup app
 *
 * Synchronizes the physical encoder zero position with the theoretical
 * step count after homing. Disables PID axes whose encoders fail to
 * calibrate.
 * 回零后将物理编码器零点与理论步数同步, 标定失败的编码器会禁用对应 PID 轴。
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
