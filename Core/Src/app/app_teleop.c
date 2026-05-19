/**
 * @file    app_teleop.c
 * @brief   PS2 joystick teleop implementation. / PS2 手柄遥操作实现。
 * @ingroup app
 */

#include "app/app_teleop.h"
#include "bsp/bsp_gripper.h"
#include "bsp/bsp_ps2.h"
#include "control/ctrl_planner.h"
#include "control/ctrl_motion_engine.h"
#include "robot_config.h"
#include <stdio.h>
#include <stdlib.h>

static SystemMode_t s_mode          = SYS_MODE_GCODE;
static uint32_t     s_last_poll_ms  = 0U;
static uint16_t     s_last_buttons  = 0xFFFFU;
static bool         s_fault_reported = false;

SystemMode_t App_Teleop_GetMode(void)        { return s_mode; }
void         App_Teleop_SetMode(SystemMode_t m) { s_mode = m; }

static void StopMotion(void)
{
    if (Ctrl_MotionEngine_GetQueueCount() > 0U)
        Ctrl_MotionEngine_Clear();
}

void App_Teleop_Init(void)
{
    s_last_poll_ms  = HAL_GetTick();
    s_fault_reported = false;
    printf("App_Teleop initialized.\r\n");
}

void App_Teleop_Task(void)
{
    if ((HAL_GetTick() - s_last_poll_ms) < PS2_POLL_INTERVAL_MS)
        return;
    s_last_poll_ms = HAL_GetTick();

    PS2_Data_t ps2 = {0};
    bool valid = BSP_PS2_ReadData(&ps2);

    if (!valid) {
        if (s_mode == SYS_MODE_PS2) {
            StopMotion();
            if (!s_fault_reported) {
                printf("Warning: PS2 analog mode lost. Motion stopped.\r\n");
                s_fault_reported = true;
            }
        }
        return;
    }
    s_fault_reported = false;

    /* SELECT button: toggle mode / SELECT 键: 切换模式 */
    if ((s_last_buttons & PS2_BTN_SELECT) && !(ps2.buttons & PS2_BTN_SELECT)) {
        if (Ctrl_MotionEngine_GetQueueCount() == 0U && !Ctrl_MotionEngine_IsRunning()) {
            s_mode = (s_mode == SYS_MODE_GCODE) ? SYS_MODE_PS2 : SYS_MODE_GCODE;
            printf("\r\n>>> MODE SWITCHED TO: [%s] <<<\r\n",
                   (s_mode == SYS_MODE_GCODE) ? "G-CODE" : "PS2 TELEOP");
        } else {
            printf("Warning: Wait for motors to stop before switching mode!\r\n");
        }
    }

    if (s_mode == SYS_MODE_PS2) {
        float dx = 0.0f, dy = 0.0f, dz = 0.0f;

        int jly = 128 - ps2.LY;
        int jlx = ps2.LX - 128;
        int jry = 128 - ps2.RY;

        if (abs(jlx) > JOYSTICK_DEADZONE) dx = (jlx / 128.0f) * TELEOP_MAX_STEP_MM;
        if (abs(jly) > JOYSTICK_DEADZONE) dy = (jly / 128.0f) * TELEOP_MAX_STEP_MM;
        if (abs(jry) > JOYSTICK_DEADZONE) dz = (jry / 128.0f) * TELEOP_MAX_STEP_MM;

        if (dx != 0.0f || dy != 0.0f || dz != 0.0f) {
            Ctrl_Planner_TeleopStep(dx, dy, dz);
        } else {
            StopMotion();
        }

        /* Cross = close, Square = open / 叉键=关闭, 方键=打开 */
        if ((s_last_buttons & PS2_BTN_CROSS) && !(ps2.buttons & PS2_BTN_CROSS)) {
            BSP_Gripper_Close(BSP_Gripper_GetHandle());
            printf("PS2: Gripper Close\r\n");
        }
        if ((s_last_buttons & PS2_BTN_SQUARE) && !(ps2.buttons & PS2_BTN_SQUARE)) {
            BSP_Gripper_Open(BSP_Gripper_GetHandle());
            printf("PS2: Gripper Open\r\n");
        }
    }

    s_last_buttons = ps2.buttons;
}
