/**
 * @file app_teleop.c
 * @brief PS2 遥操作：摇杆死区、增量命令和夹爪按键边沿处理。
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
/* 切模式、回零恢复或异常后，必须重新观察一次有效的回中/松键数据。
 * 这只是输入接管条件，不替代SafetyService的回零和故障许可。 */
static bool         s_input_armed = false;
static bool         s_valid_input_seen = false;
static uint32_t     s_acquisition_started_ms = 0U;

static void ResetInputGate(void)
{
    s_input_armed = false;
    s_last_cross_pressed = false;
    s_last_square_pressed = false;
}
static void ResetInputSession(void)
{
    ResetInputGate();s_valid_input_seen=false;
    s_acquisition_started_ms=PlatformTime_NowMs();
}

SystemMode_t App_Teleop_GetMode(void)        { return s_mode; }
void App_Teleop_SetMode(SystemMode_t mode)
{
    if (mode != SYS_MODE_GCODE && mode != SYS_MODE_PS2) return;
    if (mode != s_mode) { s_mode = mode; ResetInputSession(); }
}
void App_Teleop_ToggleMode(void)
{
    App_Teleop_SetMode(s_mode == SYS_MODE_GCODE ? SYS_MODE_PS2 : SYS_MODE_GCODE);
}

static void StopForInputLoss(void)
{
    /* 丢包无法确认操作者意图：撤销当前和未来STEP，参考失效后须重新回零。 */
    ResetInputGate();
    MotionService_AbortForFault(MOTION_FAULT_INPUT_LOST);
    CommandService_AbortPending();
}

void App_Teleop_Init(void)
{
    s_last_poll_ms  = PlatformTime_NowMs();
    ResetInputSession();
    s_fault_reported = false;
    printf("# App_Teleop initialized.\r\n");
}

void App_Teleop_Task(void)
{
    /* 即使从独立模块调用，也不能绕过AppRuntime的故障同步和动作许可。 */
    if (!MotionService_IsActionAllowed()) { ResetInputSession(); return; }
    /* G-code期间不调用软件时序PS2驱动，避免无关轮询/重初始化占用主循环。
     * 切回PS2会重新等待中立输入，不能依赖G-code期间记下的旧按键边沿。 */
    if (s_mode != SYS_MODE_PS2) return;
    uint32_t now = PlatformTime_NowMs();
    if ((now - s_last_poll_ms) < PS2_POLL_INTERVAL_MS)
        return;
    s_last_poll_ms = now;

    AppTeleopInput_t ps2 = {0};
    bool valid = AppHardware_ReadTeleopInput(&ps2);

    if (!valid) {
        /* 数字模式/配置中的false不是可用摇杆数据。首次会话只在完全空闲时
         * 给驱动有限的协商窗口，不生成STEP或夹爪动作，也不延长截止。
         * 一旦见过有效模拟帧，后续无效立即锁存，冲突按键不能重获宽限。 */
        if(!s_valid_input_seen && !s_input_armed && MotionService_IsIdle() &&
           !CommandService_IsMotionPending() && !MotionService_IsClosedLoopRecoveryActive() &&
           (uint32_t)(PlatformTime_NowMs()-s_acquisition_started_ms)<PS2_INPUT_ACQUIRE_TIMEOUT_MS)
            return;
        if (s_mode == SYS_MODE_PS2) {
            StopForInputLoss();
            if (!s_fault_reported) {
                printf("# WARN: PS2 input lost; motion fault latched; re-home required.\r\n");
                s_fault_reported = true;
            }
        }
        return;
    }
    s_valid_input_seen=true;
    s_fault_reported = false;

    /* 模式只由板端MODE键切换，SELECT不改变串口/手柄控制权。 */
    bool neutral = abs((int)ps2.left_x-128) <= JOYSTICK_DEADZONE &&
                   abs((int)ps2.left_y-128) <= JOYSTICK_DEADZONE &&
                   abs((int)ps2.right_y-128) <= JOYSTICK_DEADZONE &&
                   !ps2.cross_pressed && !ps2.square_pressed;
    if (!s_input_armed) {
        if (neutral) {
            s_input_armed = true;
            printf("# PS2: neutral input accepted; controls ready.\r\n");
        }
        return; /* 接管这一帧只确认输入，下一帧才处理新的动作。 */
    }
    if (ps2.cross_pressed && ps2.square_pressed) {
        /* 同帧开/关意图冲突：不选择任意一个、不重复覆盖PWM，先全部松开。 */
        ResetInputGate();return;
    }

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
        }
        /* 松杆只停止追加：当前帧及最多一个等待帧完成后停止，不积累长队列。
         * 每帧基准10ms；受每轴STEP上限约束时会延长，不能宣称固定20ms停步。
         * 不丢弃已受理帧，否则规划/闭环参考仍指向被丢弃的终点。 */

        /* 叉键关闭夹爪，方键打开夹爪。 */
        if (!s_last_cross_pressed && ps2.cross_pressed) {
            if (Svc_Gripper_Close()) printf("# PS2: Gripper Close\r\n");
        }
        if (!s_last_square_pressed && ps2.square_pressed) {
            if (Svc_Gripper_Open()) printf("# PS2: Gripper Open\r\n");
        }
    }

    s_last_cross_pressed = ps2.cross_pressed;
    s_last_square_pressed = ps2.square_pressed;
}
