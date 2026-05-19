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

    ErrorCode_t ok1 = BSP_AS5600_SetZero(BSP_AS5600_GetM1());
    ErrorCode_t ok2 = BSP_AS5600_SetZero(BSP_AS5600_GetM2());
    ErrorCode_t ok3 = BSP_AS5600_SetZero(BSP_AS5600_GetM3());

    printf("[Encoder] M1(I2C1):%s M2(I2C2):%s M3(I2C3):%s\r\n",
           ok1 == ERR_OK ? "OK" : "FAIL",
           ok2 == ERR_OK ? "OK" : "FAIL",
           ok3 == ERR_OK ? "OK" : "FAIL");

    Ctrl_ClosedLoop_SetAxisEnabled(0, ok1 == ERR_OK);
    Ctrl_ClosedLoop_SetAxisEnabled(1, ok2 == ERR_OK);
    Ctrl_ClosedLoop_SetAxisEnabled(2, ok3 == ERR_OK);
}
