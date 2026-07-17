/**
 * @file    app_calibration.c
 * @brief   Encoder zero-point calibration implementation. / 缂栫爜鍣ㄩ浂鐐规牎鍑嗗疄鐜般�?
 * @ingroup app
 */

#include "app/app_calibration.h"
#include "service/control/ctrl_closed_loop.h"
#include "motion_service.h"
#include "os/os_adapter.h"
#include "error_code.h"
#include <stdio.h>

bool App_Calibration_Execute(void)
{
    MotionService_ResetTheorySteps();

    /* Retry calibration up to 3 times in case of transient I2C failure.
       閲嶈瘯鏈€澶?3 �? 闃叉�?I2C 鐬€佹晠闅滃鑷磋鍒ゃ€?*/
    ErrorCode_t ok1 = ERR_ENCODER_FAIL, ok2 = ERR_ENCODER_FAIL, ok3 = ERR_ENCODER_FAIL;
    for (int retry = 0; retry < 3; retry++) {
        if (ok1 != ERR_OK) ok1 = Ctrl_ClosedLoop_SetAxisZero(0);
        if (ok2 != ERR_OK) ok2 = Ctrl_ClosedLoop_SetAxisZero(1);
        if (ok3 != ERR_OK) ok3 = Ctrl_ClosedLoop_SetAxisZero(2);
        if (ok1 == ERR_OK && ok2 == ERR_OK && ok3 == ERR_OK) break;
        Os_DelayMs(10U);
    }

    printf("[Encoder] M1(I2C1):%s M2(I2C2):%s M3(I2C3):%s\r\n",
           ok1 == ERR_OK ? "OK" : "FAIL",
           ok2 == ERR_OK ? "OK" : "FAIL",
           ok3 == ERR_OK ? "OK" : "FAIL");

    /* Enable all axes regardless of calibration result.
       A failed SetZero just means the zero-offset wasn't set �?the encoder
       may still work for relative positioning and multi-turn tracking.
       鏃犺鏍″噯鏄惁鎴愬姛閮藉惎鐢ㄦ墍鏈夎酱銆係etZero 澶辫触浠呰〃绀洪浂鐐规湭璁剧疆,
       缂栫爜鍣ㄤ粛鍙敤浜庣浉瀵瑰畾浣嶅拰澶氬湀杩借釜�?*/
    bool success = (ok1 == ERR_OK && ok2 == ERR_OK && ok3 == ERR_OK);
    Ctrl_ClosedLoop_SetAxisEnabled(0, success);
    Ctrl_ClosedLoop_SetAxisEnabled(1, success);
    Ctrl_ClosedLoop_SetAxisEnabled(2, success);
    return success;
}
