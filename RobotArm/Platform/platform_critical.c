/**
 * @file platform_critical.c
 * @brief 临界区适配：保存并恢复原中断状态，保护短时间共享数据复制。
 */
#include "platform_critical.h"

#include "main.h"

PlatformCriticalState_t PlatformCritical_Enter(void)
{
    PlatformCriticalState_t state = __get_PRIMASK();
    __disable_irq();
    return state;
}

void PlatformCritical_Exit(PlatformCriticalState_t state)
{
    __set_PRIMASK(state);
}
