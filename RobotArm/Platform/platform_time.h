#ifndef ROBOTARM_PLATFORM_TIME_H
#define ROBOTARM_PLATFORM_TIME_H

#include <stdint.h>

uint32_t PlatformTime_NowMs(void);
void PlatformTime_Init(void);
uint32_t PlatformTime_NowUs(void);

#endif
