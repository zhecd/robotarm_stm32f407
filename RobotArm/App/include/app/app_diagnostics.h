/**
 * @file app_diagnostics.h
 * @brief 只读诊断流程：采集原始编码器读数，处理查询，保持执行器停用。
 */
#ifndef ROBOTARM_APP_DIAGNOSTICS_H
#define ROBOTARM_APP_DIAGNOSTICS_H
#include <stdbool.h>
/** 当前诊断构建是否轮询编码器；关闭时M114的VALID为0，不代表传感器故障。 */
bool AppDiagnostics_EncoderSamplingEnabled(void);
void AppDiagnostics_Init(void);
void AppDiagnostics_RunOnce(void);
#endif
