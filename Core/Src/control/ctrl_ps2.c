/**
 * @file    ctrl_ps2.c
 * @brief   PS2 controller abstraction implementation / PS2 手柄抽象实现
 * @ingroup control
 */

#include "control/ctrl_ps2.h"

void Ctrl_PS2_Init(void)
{
    BSP_PS2_Init();
}

bool Ctrl_PS2_ReadData(PS2_Data_t *data)
{
    return BSP_PS2_ReadData(data);
}

bool Ctrl_PS2_IsAnalogMode(void)
{
    return BSP_PS2_IsAnalogMode();
}
