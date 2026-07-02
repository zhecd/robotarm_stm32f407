/**
 * @file    app_calibration.c
 * @brief   Encoder zero-point calibration implementation. / 编码器零点校准实现。
 * @ingroup app
 */

#include "app/app_calibration.h"
#include "control/ctrl_motion_engine.h"
#include "control/ctrl_closed_loop.h"
#include "bsp/bsp_as5600.h"
#include "error_code.h"
#include <stdio.h>

void App_Calibration_Execute(void)
{
    Ctrl_MotionEngine_ResetTheorySteps();

    /* Retry calibration up to 3 times in case of transient I2C failure.
       重试最多 3 次, 防止 I2C 瞬态故障导致误判。 */
    ErrorCode_t ok1 = ERR_ENCODER_FAIL, ok2 = ERR_ENCODER_FAIL, ok3 = ERR_ENCODER_FAIL;
    for (int retry = 0; retry < 3; retry++) {
        if (ok1 != ERR_OK) ok1 = BSP_AS5600_SetZero(BSP_AS5600_GetM1());
        if (ok2 != ERR_OK) ok2 = BSP_AS5600_SetZero(BSP_AS5600_GetM2());
        if (ok3 != ERR_OK) ok3 = BSP_AS5600_SetZero(BSP_AS5600_GetM3());
        if (ok1 == ERR_OK && ok2 == ERR_OK && ok3 == ERR_OK) break;
        HAL_Delay(10);
    }

    printf("[Encoder] M1(I2C1):%s M2(I2C2):%s M3(I2C3):%s\r\n",
           ok1 == ERR_OK ? "OK" : "FAIL",
           ok2 == ERR_OK ? "OK" : "FAIL",
           ok3 == ERR_OK ? "OK" : "FAIL");

    /* Enable all axes regardless of calibration result.
       A failed SetZero just means the zero-offset wasn't set — the encoder
       may still work for relative positioning and multi-turn tracking.
       无论校准是否成功都启用所有轴。SetZero 失败仅表示零点未设置,
       编码器仍可用于相对定位和多圈追踪。 */
    Ctrl_ClosedLoop_SetAxisEnabled(0, true);
    Ctrl_ClosedLoop_SetAxisEnabled(1, true);
    Ctrl_ClosedLoop_SetAxisEnabled(2, true);
}
