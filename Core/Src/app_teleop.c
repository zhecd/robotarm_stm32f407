#include "app_teleop.h"
#include "bsp_gripper.h"
#include "bsp_ps2.h"
#include "motion_planner.h"
#include "motor_core.h"
#include <stdio.h>
#include <stdlib.h>

#define PS2_POLL_INTERVAL_MS 5U
#define JOYSTICK_DEADZONE    12
#define TELEOP_MAX_STEP_MM   0.45f

SystemMode_t current_sys_mode = SYS_MODE_GCODE;

static uint32_t last_ps2_read_time = 0U;
static uint16_t last_buttons = 0xFFFFU;
static bool last_ps2_fault_reported = false;

static void App_Teleop_StopMotionSafely(void)
{
    if (Motor_Buffer_GetCount() > 0U) {
        Motor_Buffer_Clear();
    }
}

void App_Teleop_Init(void)
{
    last_ps2_read_time = HAL_GetTick();
    last_ps2_fault_reported = false;
    printf("App_Teleop initialized.\r\n");
}

void App_Teleop_Task(void)
{
    if ((HAL_GetTick() - last_ps2_read_time) < PS2_POLL_INTERVAL_MS) {
        return;
    }
    last_ps2_read_time = HAL_GetTick();

    PS2_Data_t ps2_data = {0};
    bool is_ps2_valid = BSP_PS2_ReadData(&ps2_data);

    if (!is_ps2_valid) {
        if (current_sys_mode == SYS_MODE_PS2) {
            App_Teleop_StopMotionSafely();

            if (!last_ps2_fault_reported) {
                printf("Warning: PS2 analog mode lost or controller data invalid. Motion stopped.\r\n");
                printf("Please switch the controller back to Analog mode before teleop.\r\n");
                last_ps2_fault_reported = true;
            }
        }
        return;
    }

    last_ps2_fault_reported = false;

    if ((last_buttons & PS2_BTN_SELECT) && !(ps2_data.buttons & PS2_BTN_SELECT)) {
        if ((Motor_Buffer_GetCount() == 0U) && !Motor_Core_IsRunning()) {
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

        int joy_ly = 128 - ps2_data.LY;
        int joy_lx = ps2_data.LX - 128;
        int joy_ry = 128 - ps2_data.RY;

        if (abs(joy_lx) > JOYSTICK_DEADZONE) dx = (joy_lx / 128.0f) * TELEOP_MAX_STEP_MM;
        if (abs(joy_ly) > JOYSTICK_DEADZONE) dy = (joy_ly / 128.0f) * TELEOP_MAX_STEP_MM;
        if (abs(joy_ry) > JOYSTICK_DEADZONE) dz = (joy_ry / 128.0f) * TELEOP_MAX_STEP_MM;

        if ((dx != 0.0f) || (dy != 0.0f) || (dz != 0.0f)) {
            Motion_Planner_TeleopStep(dx, dy, dz);
        } else {
            App_Teleop_StopMotionSafely();
        }

        if ((last_buttons & PS2_BTN_CROSS) && !(ps2_data.buttons & PS2_BTN_CROSS)) {
            BSP_Gripper_Close(&hgripper);
            printf("PS2: Gripper Close (X Pressed)\r\n");
        }

        if ((last_buttons & PS2_BTN_SQUARE) && !(ps2_data.buttons & PS2_BTN_SQUARE)) {
            BSP_Gripper_Open(&hgripper);
            printf("PS2: Gripper Open (Square Pressed)\r\n");
        }
    }

    last_buttons = ps2_data.buttons;
}
