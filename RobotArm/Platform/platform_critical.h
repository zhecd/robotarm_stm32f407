#ifndef ROBOTARM_PLATFORM_CRITICAL_H
#define ROBOTARM_PLATFORM_CRITICAL_H

#include <stdint.h>

typedef uint32_t PlatformCriticalState_t;

PlatformCriticalState_t PlatformCritical_Enter(void);
void PlatformCritical_Exit(PlatformCriticalState_t state);

#endif
