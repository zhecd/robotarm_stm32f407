/**
 * @file app_calibration.h
 * @brief 回零后的编码器置零：有限重试，任一轴失败则禁止进入正常运行。
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
