/**
 * @file    ctrl_ps2.h
 * @brief   PS2 controller abstraction interface / PS2 手柄抽象接口
 * @ingroup control
 *
 * Thin wrapper over BSP PS2 driver. APP layer calls Ctrl_PS2_*
 * instead of BSP_PS2_* directly, preserving the APP→CONTROL→BSP
 * dependency chain.
 * BSP PS2 驱动的薄包装层, APP 层通过此接口访问手柄, 维持层次依赖。
 */

#ifndef __CTRL_PS2_H__
#define __CTRL_PS2_H__

#include "bsp/bsp_ps2.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void Ctrl_PS2_Init(void);
bool Ctrl_PS2_ReadData(PS2_Data_t *data);
bool Ctrl_PS2_IsAnalogMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_PS2_H__ */
