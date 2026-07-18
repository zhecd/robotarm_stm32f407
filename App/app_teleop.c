/**
 * @file    app_teleop.c
 * @brief   PS2 joystick teleop implementation. / PS2 鎵嬫焺閬ユ搷浣滃疄鐜般€? * @ingroup app
 */

#include "app/app_teleop.h"
#include "app_hardware_adapter.h"
#include "service/svc_gripper.h"
#include "command_service.h"
#include "motion_service.h"
#include "platform_time.h"
#include "robot_config.h"
#include <stdio.h>
#include <stdlib.h>

static SystemMode_t s_mode           = SYS_MODE_GCODE;
static uint32_t     s_last_poll_ms   = 0U;
static bool         s_last_cross_pressed = false;
static bool         s_last_square_pressed = false;
static bool         s_fault_reported = false;

SystemMode_t App_Teleop_GetMode(void)        { return s_mode; }
void         App_Teleop_SetMode(SystemMode_t m) { s_mode = m; }
void         App_Teleop_ToggleMode(void)       { s_mode = (s_mode == SYS_MODE_GCODE) ? SYS_MODE_PS2 : SYS_MODE_GCODE; }

static void StopMotion(void)
{
    if (MotionService_GetQueueCount() > 0U)
        MotionService_ClearQueuedFrames();
}

void App_Teleop_Init(void)
{
    s_last_poll_ms  = PlatformTime_NowMs();
    s_last_cross_pressed = false;
    s_last_square_pressed = false;
    s_fault_reported = false;
    printf("App_Teleop initialized.\r\n");
}

void App_Teleop_Task(void)
{
    uint32_t now = PlatformTime_NowMs();
    if ((now - s_last_poll_ms) < PS2_POLL_INTERVAL_MS)
        return;
    s_last_poll_ms = now;

    AppTeleopInput_t ps2 = {0};
    bool valid = AppHardware_ReadTeleopInput(&ps2);

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

    /* SELECT button: toggle mode / SELECT �? 鍒囨崲妯″紡 */
    /* Mode changes are owned exclusively by the physical MODE key.  Keeping
       SELECT out of this path prevents packet noise or an accidental press
       from silently enabling/disabling UART G-code control. */

    if (s_mode == SYS_MODE_PS2) {
        float dx = 0.0f, dy = 0.0f, dz = 0.0f;

        int jly = 128 - ps2.left_y;
        int jlx = ps2.left_x - 128;
        int jry = 128 - ps2.right_y;

        if (abs(jlx) > JOYSTICK_DEADZONE) dx = (jlx / 128.0f) * TELEOP_MAX_STEP_MM;
        if (abs(jly) > JOYSTICK_DEADZONE) dy = (jly / 128.0f) * TELEOP_MAX_STEP_MM;
        if (abs(jry) > JOYSTICK_DEADZONE) dz = (jry / 128.0f) * TELEOP_MAX_STEP_MM;

        if (dx != 0.0f || dy != 0.0f || dz != 0.0f) {
            (void)CommandService_RunTeleopStep(dx, dy, dz);
        } else {
            StopMotion();
        }

        /* Cross = close, Square = open / 鍙夐�?鍏抽�? 鏂归�?鎵撳�?*/
        if (!s_last_cross_pressed && ps2.cross_pressed) {
            Svc_Gripper_Close();
            printf("PS2: Gripper Close\r\n");
        }
        if (!s_last_square_pressed && ps2.square_pressed) {
            Svc_Gripper_Open();
            printf("PS2: Gripper Open\r\n");
        }
    }

    s_last_cross_pressed = ps2.cross_pressed;
    s_last_square_pressed = ps2.square_pressed;
}
