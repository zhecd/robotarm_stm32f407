#include "app_hardware_adapter.h"

#include "main.h"
#include "tim.h"
#include "usart.h"

#include "bsp/bsp_led.h"
#include "bsp/bsp_uart1.h"
#include "device/dev_gripper.h"
#include "device/dev_input.h"
#include "device/dev_joint.h"
#include "device/dev_limit_switch.h"

void AppHardware_Init(void)
{
    BSP_LED_Init();
    Dev_Joint_Init();
    BSP_UART1_Init();
    Dev_Input_Init();
    Dev_Gripper_Init(&htim2, TIM_CHANNEL_2);
}

void AppHardware_ConfigureMotionDrivers(void)
{
    Dev_Joint_EnableAll(true);
    Dev_Joint_ConfigureDrivers(&huart6);
}

void AppHardware_EnableJoints(bool enabled)
{
    Dev_Joint_EnableAll(enabled);
}

void AppHardware_SetModeIndicator(bool gcode_on, bool ps2_on)
{
    BSP_LED_SetState(LED_0, gcode_on ? LED_ON : LED_OFF);
    BSP_LED_SetState(LED_1, ps2_on ? LED_ON : LED_OFF);
}

void AppHardware_StartStepTimer(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
}

bool AppHardware_IsLimitTriggered(uint8_t joint_index)
{
    if (joint_index >= DEV_JOINT_COUNT) return false;
    return Dev_LimitSwitch_IsTriggered((DevJointId_t)joint_index);
}

bool AppHardware_ReadTeleopInput(AppTeleopInput_t *out_input)
{
    if (!out_input) return false;

    DevInputState_t input = {0};
    if (!Dev_Input_Read(&input)) return false;

    out_input->left_x = input.left_x;
    out_input->left_y = input.left_y;
    out_input->right_y = input.right_y;
    out_input->cross_pressed = (input.buttons & DEV_INPUT_BTN_CROSS) == 0U;
    out_input->square_pressed = (input.buttons & DEV_INPUT_BTN_SQUARE) == 0U;
    return true;
}

void AppHardware_SendText(const char *text) { BSP_UART1_SendString(text); }
bool AppHardware_TakeLineTimeout(void) { return BSP_UART1_TakeLineTimeout(); }
bool AppHardware_TakeRxOverflow(void) { return BSP_UART1_TakeRxOverflow(); }
bool AppHardware_TakeTxOverflow(void) { return BSP_UART1_TakeTxOverflow(); }
bool AppHardware_ReadLine(char *out_line, size_t out_size) { return BSP_UART1_ReadLine(out_line, out_size); }
void AppHardware_DiscardRx(void) { BSP_UART1_DiscardRx(); }
