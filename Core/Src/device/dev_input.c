/** @file dev_input.c @brief PS2-based operator-input adapter. */
#include "device/dev_input.h"
#include "driver/drv_ps2.h"

void Dev_Input_Init(void)
{
    Drv_PS2_Init();
}

bool Dev_Input_Read(DevInputState_t *state)
{
    Ps2Input_t raw;
    if (state == NULL || !Drv_PS2_ReadData(&raw)) return false;
    state->buttons = raw.buttons;
    state->left_x = raw.LX;
    state->left_y = raw.LY;
    state->right_x = raw.RX;
    state->right_y = raw.RY;
    return true;
}

bool Dev_Input_IsAnalogMode(void)
{
    return Drv_PS2_IsAnalogMode();
}
