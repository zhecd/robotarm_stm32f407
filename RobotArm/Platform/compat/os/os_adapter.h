/**
 * @file os_adapter.h
 * @brief Bare-metal operating-system abstraction.
 *
 * The public interface remains stable when a future FreeRTOS port replaces
 * the current superloop implementation.
 */
#ifndef ROBOTARM_OS_ADAPTER_H
#define ROBOTARM_OS_ADAPTER_H

#include <stdint.h>

uint32_t Os_GetTickMs(void);
void Os_DelayMs(uint32_t delay_ms);
void Os_DelayUs(uint32_t delay_us);

#endif /* ROBOTARM_OS_ADAPTER_H */
