/**
 * @file app_calibration.c
 * @brief 回零后的编码器置零：有限重试，任一轴失败则禁止进入正常运行。
 */

#include "app/app_calibration.h"
#include "motion_service.h"
#include "os/os_adapter.h"
#include "error_code.h"
#include <stdio.h>

bool App_Calibration_Execute(void)
{
    MotionService_ResetTheorySteps();

    /* 每轴最多尝试置零 3 次；只重试失败的轴，防止反复改变已成功轴的零偏。 */
    ErrorCode_t ok1 = ERR_ENCODER_FAIL, ok2 = ERR_ENCODER_FAIL, ok3 = ERR_ENCODER_FAIL;
    for (int retry = 0; retry < 3; retry++) {
        if (ok1 != ERR_OK) ok1 = MotionService_SetClosedLoopAxisZero(0);
        if (ok2 != ERR_OK) ok2 = MotionService_SetClosedLoopAxisZero(1);
        if (ok3 != ERR_OK) ok3 = MotionService_SetClosedLoopAxisZero(2);
        if (ok1 == ERR_OK && ok2 == ERR_OK && ok3 == ERR_OK) break;
        Os_DelayMs(10U);
    }

    printf("# [Encoder] M1(I2C1):%s M2(I2C2):%s M3(I2C3):%s\r\n",
           ok1 == ERR_OK ? "OK" : "FAIL",
           ok2 == ERR_OK ? "OK" : "FAIL",
           ok3 == ERR_OK ? "OK" : "FAIL");

    /* 只有 3 轴全部标定成功才启用反馈；失败时 AppRuntime 保持运动禁止。 */
    bool success = (ok1 == ERR_OK && ok2 == ERR_OK && ok3 == ERR_OK);
    MotionService_SetClosedLoopAxisEnabled(0, success);
    MotionService_SetClosedLoopAxisEnabled(1, success);
    MotionService_SetClosedLoopAxisEnabled(2, success);
    return success;
}
