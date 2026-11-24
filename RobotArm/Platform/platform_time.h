/**
 * @file platform_time.h
 * @brief 平台时间：毫秒时基和微秒读数；业务层通过此接口替换测试时钟。
 * @details 微秒值自初始化起计时，约71.58分钟按uint32_t自然回绕；使用无符号差值。
 */
#ifndef ROBOTARM_PLATFORM_TIME_H
#define ROBOTARM_PLATFORM_TIME_H

#include <stdint.h>

uint32_t PlatformTime_NowMs(void);
void PlatformTime_Init(void);
/** 在HAL毫秒SysTick中持续累计CPU周期，初始化前调用无副作用。 */
void PlatformTime_OnTickFromISR(void);
/** 固定CPU时钟；SysTick不能连续停用超过一次DWT回绕，调试暂停不计入运行时间。 */
uint32_t PlatformTime_NowUs(void);

#endif
