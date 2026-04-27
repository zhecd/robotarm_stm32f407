#include "app_teleop.h"
#include "bsp_ps2.h"
#include "motion_planner.h"
#include "motor_core.h"
#include "bsp_gripper.h"
#include <stdio.h>
#include <stdlib.h>

#define PS2_POLL_INTERVAL_MS 5U
#define JOYSTICK_DEADZONE    12
#define TELEOP_MAX_STEP_MM   0.45f

SystemMode_t current_sys_mode = SYS_MODE_GCODE;

static uint32_t last_ps2_read_time = 0;
static uint16_t last_buttons = 0xFFFF;

void App_Teleop_Init(void)
{
    last_ps2_read_time = HAL_GetTick();
    printf("App_Teleop initialized.\r\n");
}

void App_Teleop_Task(void)
{
    if (HAL_GetTick() - last_ps2_read_time < PS2_POLL_INTERVAL_MS) {
        return;
    }
    last_ps2_read_time = HAL_GetTick();

    PS2_Data_t my_ps2 = {0};
    bool is_ps2_connected = BSP_PS2_ReadData(&my_ps2);

    if (!is_ps2_connected) {
        return;
    }

    if ((last_buttons & PS2_BTN_SELECT) && !(my_ps2.buttons & PS2_BTN_SELECT)) {
        if ((Motor_Buffer_GetCount() == 0) && !Motor_Core_IsRunning()) {
            current_sys_mode = (current_sys_mode == SYS_MODE_GCODE) ? SYS_MODE_PS2 : SYS_MODE_GCODE;
            printf("\r\n>>> MODE SWITCHED TO: [%s] <<<\r\n",
                   (current_sys_mode == SYS_MODE_GCODE) ? "G-CODE" : "PS2 TELEOP");
        } else {
            printf("Warning: Please wait for motors to stop before switching mode!\r\n");
        }
    }

    if (current_sys_mode == SYS_MODE_PS2) {
        float dx = 0.0f;
        float dy = 0.0f;
        float dz = 0.0f;

        int joy_ly = 128 - my_ps2.LY;
        int joy_lx = my_ps2.LX - 128;
        int joy_ry = 128 - my_ps2.RY;

        if (abs(joy_lx) > JOYSTICK_DEADZONE) dx = (joy_lx / 128.0f) * TELEOP_MAX_STEP_MM;
        if (abs(joy_ly) > JOYSTICK_DEADZONE) dy = (joy_ly / 128.0f) * TELEOP_MAX_STEP_MM;
        if (abs(joy_ry) > JOYSTICK_DEADZONE) dz = (joy_ry / 128.0f) * TELEOP_MAX_STEP_MM;

        if ((dx != 0.0f) || (dy != 0.0f) || (dz != 0.0f)) {
            Motion_Planner_TeleopStep(dx, dy, dz);
        } else if (Motor_Buffer_GetCount() > 0) {
            Motor_Buffer_Clear();
        }

        if ((last_buttons & PS2_BTN_CROSS) && !(my_ps2.buttons & PS2_BTN_CROSS)) {
            BSP_Gripper_Close(&hgripper);
            printf("PS2: Gripper Close (X Pressed)\r\n");
        }

        if ((last_buttons & PS2_BTN_SQUARE) && !(my_ps2.buttons & PS2_BTN_SQUARE)) {
            BSP_Gripper_Open(&hgripper);
            printf("PS2: Gripper Open (Square Pressed)\r\n");
        }
    }

    last_buttons = my_ps2.buttons;
}
