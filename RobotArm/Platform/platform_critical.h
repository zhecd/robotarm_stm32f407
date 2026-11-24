/**
 * @file platform_critical.h
 * @brief 临界区适配：保存并恢复原中断状态，保护短时间共享数据复制。
 */
#ifndef ROBOTARM_PLATFORM_CRITICAL_H
#define ROBOTARM_PLATFORM_CRITICAL_H

#include <stdint.h>

typedef uint32_t PlatformCriticalState_t;

PlatformCriticalState_t PlatformCritical_Enter(void);
void PlatformCritical_Exit(PlatformCriticalState_t state);

#endif
