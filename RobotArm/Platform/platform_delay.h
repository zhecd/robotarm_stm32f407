/**
 * @file platform_delay.h
 * @brief 裸机延时：属于阻塞操作，仅在初始化等允许阻塞的路径使用。
 */
#ifndef ROBOTARM_PLATFORM_DELAY_H
#define ROBOTARM_PLATFORM_DELAY_H

#include <stdint.h>

void PlatformDelay_Ms(uint32_t delay_ms);
void PlatformDelay_Us(uint32_t delay_us);

#endif
