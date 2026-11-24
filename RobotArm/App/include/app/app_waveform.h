/**
 * @file app_waveform.h
 * @brief 独立波形测试入口；不配置 TMC、不使能电机、不初始化夹爪或回零。
 */
#ifndef ROBOTARM_APP_WAVEFORM_H
#define ROBOTARM_APP_WAVEFORM_H
#include <stdint.h>
/** 实验室SWD采样控制字，bit0..2对应ENC1..3；初始化为7。
 * 仅独立WAVE诊断使用，不是普通控制接口。非7时禁止新波形并标记未采样反馈无效。
 */
extern volatile uint32_t g_waveform_encoder_poll_mask;
uint8_t AppWaveform_EncoderPollMask(void);
void AppWaveform_Init(void);
void AppWaveform_RunOnce(void);
/** 已收完整行的主循环入口；严格命令格式，拒绝普通动作与恢复命令。 */
void AppWaveform_HandleCommand(const char *line);
#endif
