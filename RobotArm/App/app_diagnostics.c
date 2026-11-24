/**
 * @file app_diagnostics.c
 * @brief 只读诊断流程：采集原始编码器读数，处理查询，保持执行器停用。
 */
#include "app/app_diagnostics.h"
#include "app/app_serial_protocol.h"
#include "app_hardware_adapter.h"
#include "platform_time.h"
#include "platform_critical.h"
#include "state_service.h"
#include "safety_service.h"
#include "limit_observation_service.h"
#include <math.h>
#ifndef ROBOTARM_DIAGNOSTIC_ENCODERS
#define ROBOTARM_DIAGNOSTIC_ENCODERS 1
#endif
static uint32_t s_last_sample_ms;
static uint8_t ReadLimits(void)
{
    uint8_t mask=0U;
    for (uint8_t axis=0;axis<3;axis++)
        if (AppHardware_IsLimitTriggered(axis)) mask|=(uint8_t)(1U<<axis);
    return mask;
}
bool AppDiagnostics_EncoderSamplingEnabled(void)
{
    return ROBOTARM_DIAGNOSTIC_ENCODERS!=0;
}
void AppDiagnostics_Init(void)
{
    PlatformTime_Init();
    AppHardware_InitDiagnostics();
    StateService_Init();
    SafetyService_Init();
    /* GPIO初始化即可能收到中断。早期回调也可限流，防止尚未进入主循环就被占满。
     * 此处原子重置观察器和三路IRQ，避免早期锁存被清零后仍留有隐藏的IMR屏蔽。 */
    PlatformCriticalState_t key=PlatformCritical_Enter();
    LimitObservation_Init(ReadLimits(),PlatformTime_NowMs());
    AppHardware_ResetDiagnosticLimitInterrupts();
    PlatformCritical_Exit(key);
    AppSerial_Init(APP_SERIAL_DIAGNOSTIC);
    s_last_sample_ms=PlatformTime_NowMs()-20U;
    AppHardware_SendText("# DIAGNOSTIC: actuators disabled; read-only commands\r\n");
}
void AppDiagnostics_RunOnce(void)
{
    uint32_t now=PlatformTime_NowMs();
    LimitObservation_Sample(ReadLimits(),now);
    if (AppDiagnostics_EncoderSamplingEnabled() && (uint32_t)(now-s_last_sample_ms)>=20U) {
        s_last_sample_ms=now;
        for (uint8_t axis=0;axis<ARM_AXIS_COUNT;axis++) {
            float angle;
            uint32_t started=PlatformTime_NowMs();
            if (AppHardware_ReadMotorAngle(axis,&angle) && isfinite(angle))
                StateService_PublishAxisSampleTimed(axis,angle,0.0f,started,PlatformTime_NowMs());
            else StateService_PublishAxisReadFailure(axis,1U);
        }
    }
    AppSerial_Service();
}
