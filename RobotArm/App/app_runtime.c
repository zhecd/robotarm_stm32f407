#include "app_init.h"

#include "app_hardware_adapter.h"
#include "app/app_runtime_events.h"
#include "service/svc_gripper.h"
#include "service/svc_homing.h"
#include "motion_service.h"
#include "platform_delay.h"
#include "platform_time.h"
#include "safety_service.h"
#include "state_service.h"
#include "app/app_calibration.h"
#include "app/app_gcode_exec.h"
#include "app/app_gcode_parser.h"
#include "app/app_teleop.h"
#include "common/robot_home_pose.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

static volatile bool s_mode_switch_requested = false;
static bool s_wait_for_motion = false;
static bool s_wait_for_plan_start = false;

typedef enum {
    APP_BOOT_HOMING = 0,
    APP_BOOT_READY,
    APP_BOOT_FAILED
} AppBootState_t;

static AppBootState_t s_boot_state = APP_BOOT_HOMING;
static bool s_homing_is_recovery = false;

static void App_StartHoming(bool recovery);

void AppRuntime_RequestModeSwitch(void)
{
    s_mode_switch_requested = true;
}

static void App_EncoderReadTask(void)
{
    static uint32_t last_tick = 0U;
    uint32_t now = PlatformTime_NowMs();
    if ((now - last_tick) < 20U) return;
    last_tick = now;

    for (int i = 0; i < (int)MOTION_AXIS_COUNT; i++) {
        if (MotionService_IsClosedLoopAxisEnabled(i)) {
            float angle;
            MotionService_GetClosedLoopAxisAngle(i, &angle);
        }
    }
}

static void App_EncoderReportTask(void)
{
    static uint32_t last_tick = 0U;
    static float last[MOTION_AXIS_COUNT];
    static bool first = true;
    StateServiceStatus_t state;
    uint32_t now = PlatformTime_NowMs();

    if ((now - last_tick) < 1000U) return;
    last_tick = now;

    StateService_GetStatus(&state);
    float diff = fabsf(state.motor_angle_deg[0] - last[0]) +
                 fabsf(state.motor_angle_deg[1] - last[1]) +
                 fabsf(state.motor_angle_deg[2] - last[2]);
    if (first || diff > 1.0f) {
        first = false;
        printf("# ENC M1:%.1f M2:%.1f M3:%.1f\r\n",
               state.motor_angle_deg[0], state.motor_angle_deg[1], state.motor_angle_deg[2]);
        for (int i = 0; i < (int)MOTION_AXIS_COUNT; i++) last[i] = state.motor_angle_deg[i];
    }
}

static void App_ClosedLoopTask(void)
{
    static uint32_t last_tick = 0U;
    uint32_t now = PlatformTime_NowMs();
    if ((now - last_tick) < 20U) return;
    last_tick = now;
    MotionService_UpdateClosedLoop();
}

static void App_ReportM114(void)
{
    float x, y, z;
    StateServiceStatus_t state;
    App_GCodeExec_GetPlannedPosition(&x, &y, &z);
    StateService_GetStatus(&state);

    printf("M114 PLAN X:%.2f Y:%.2f Z:%.2f J:%.2f,%.2f,%.2f ENC:%.2f,%.2f,%.2f\r\n",
           x, y, z, state.joint_angle_deg[0], state.joint_angle_deg[1], state.joint_angle_deg[2],
           state.motor_angle_deg[0], state.motor_angle_deg[1], state.motor_angle_deg[2]);
}

static void App_ReportM119(void)
{
    printf("M119 M1:%s M2:%s M3:%s\r\n",
           AppHardware_IsLimitTriggered(0U) ? "TRIGGERED" : "OPEN",
           AppHardware_IsLimitTriggered(1U) ? "TRIGGERED" : "OPEN",
           AppHardware_IsLimitTriggered(2U) ? "TRIGGERED" : "OPEN");
}

static void App_GCodeTask(void)
{
    char line[256];
    GCodeFrame_t frame;

    if (App_Teleop_GetMode() != SYS_MODE_GCODE && !SafetyService_HasFault()) return;
    if (s_wait_for_plan_start) {
        ErrorCode_t result;
        if (!App_GCodeExec_TakeMoveResult(&result)) return;
        s_wait_for_plan_start = false;
        if (result != ERR_OK) {
            printf("error: command rejected (%d)\r\n", (int)result);
            return;
        }
        MotionService_SyncClosedLoopTarget();
        printf("ok\r\n");
        return;
    }
    if (s_wait_for_motion) {
        if (SafetyService_HasFault()) {
            s_wait_for_motion = false;
            printf("error: M400 aborted by safety fault\r\n");
        } else if (MotionService_IsIdle() && !App_GCodeExec_IsMotionPending()) {
            s_wait_for_motion = false;
            printf("ok\r\n");
        }
        return;
    }

    if (AppHardware_TakeLineTimeout()) printf("error: incomplete command; CRLF required\r\n");
    if (AppHardware_TakeRxOverflow()) printf("error: UART RX overflow; command discarded\r\n");
    if (!AppHardware_ReadLine(line, sizeof(line))) return;

    bool has_printable = false;
    for (const char *p = line; *p != '\0'; p++) {
        if (isprint((unsigned char)*p)) { has_printable = true; break; }
    }
    if (!has_printable) return;

    if (!App_GCodeParser_ParseLine(line, &frame)) {
        printf("error: Parse failed!\r\n");
        return;
    }

    if (frame.type == GCMD_M114) { App_ReportM114(); return; }
    if (frame.type == GCMD_M119) { App_ReportM119(); return; }
    if (frame.type == GCMD_M400) {
        if (SafetyService_HasFault()) printf("error: M400 rejected by safety fault\r\n");
        else if (MotionService_IsIdle() && !App_GCodeExec_IsMotionPending()) printf("ok\r\n");
        else s_wait_for_motion = true;
        return;
    }
    if (frame.type == GCMD_M999) {
        if (!SafetyService_HasFault()) {
            printf("error: M999 requires a safety fault\r\n");
            return;
        }
        App_StartHoming(true);
        printf("M999: homing started\r\n");
        return;
    }

    ErrorCode_t status = App_GCodeExec_Run(&frame);
    if (status == ERR_PENDING) {
        s_wait_for_plan_start = true;
        return;
    }
    if (status != ERR_OK) {
        printf("error: command rejected (%d)\r\n", (int)status);
        return;
    }
    if (frame.type == GCMD_G0 || frame.type == GCMD_G1) MotionService_SyncClosedLoopTarget();
    switch (frame.type) {
    case GCMD_M3: printf("M3OK\r\n"); break;
    case GCMD_M5: printf("M5OK\r\n"); break;
    default: printf("ok\r\n"); break;
    }
}

static void App_ModeSwitchTask(void)
{
    static uint32_t last_switch_ms = 0U;
    if (!s_mode_switch_requested) return;
    s_mode_switch_requested = false;

    uint32_t now = PlatformTime_NowMs();
    if ((now - last_switch_ms) < 200U) return;
    last_switch_ms = now;
    if (!MotionService_IsIdle() || App_GCodeExec_IsMotionPending()) {
        printf("# WARN: Motors moving, cannot switch mode!\r\n");
        return;
    }
    App_Teleop_ToggleMode();
    SystemMode_t mode = App_Teleop_GetMode();
    AppHardware_SetModeIndicator(mode == SYS_MODE_GCODE, mode == SYS_MODE_PS2);
    printf("# MODE: %s\r\n", mode == SYS_MODE_GCODE ? "G-CODE" : "PS2 TELEOP");
}

static void App_UpdateModeIndicator(SystemMode_t mode)
{
    static SystemMode_t last_mode = (SystemMode_t)-1;
    if (mode == last_mode) return;
    last_mode = mode;
    AppHardware_SetModeIndicator(mode == SYS_MODE_GCODE, mode == SYS_MODE_PS2);
}

static const char *App_MotionFaultText(MotionFaultReason_t reason)
{
    switch (reason) {
    case MOTION_FAULT_LIMIT_SWITCH: return "limit switch";
    case MOTION_FAULT_ENCODER: return "encoder communication";
    case MOTION_FAULT_SOFT_LIMIT: return "actual joint soft limit";
    case MOTION_FAULT_QUEUE_TIMEOUT: return "planner queue timeout";
    default: return "unspecified";
    }
}

static void App_StartHoming(bool recovery)
{
    s_wait_for_motion = false;
    s_wait_for_plan_start = false;
    MotionService_SetLimitMonitoring(false);
    AppHardware_EnableJoints(true);
    Svc_Homing_Start();
    s_homing_is_recovery = recovery;
    s_boot_state = APP_BOOT_HOMING;
}

/* Finalize only after the non-blocking homing state machine has completed. */
static bool App_FinalizeAfterHoming(void)
{
    MotionService_Init();
    if (App_GCodeExec_Init(g_robot_home_pose.x_mm, g_robot_home_pose.y_mm,
                           g_robot_home_pose.z_mm) != ERR_OK) {
        printf("error: initial pose is unreachable; motion disabled\r\n");
        return false;
    }

    App_Teleop_Init();
    MotionService_InitClosedLoop();
    StateService_Init();
    if (!App_Calibration_Execute()) {
        printf("error: encoder calibration failed; motion disabled\r\n");
        return false;
    }

    SafetyService_MarkHomed();
    SafetyService_ClearAfterSuccessfulHoming();
    MotionService_SetLimitMonitoring(true);
    printf("# [HomePose] XYZ=%.1f,%.1f,%.1f J=%.1f,%.1f,%.1f\r\n",
           g_robot_home_pose.x_mm, g_robot_home_pose.y_mm, g_robot_home_pose.z_mm,
           g_robot_home_pose.joint_deg[0], g_robot_home_pose.joint_deg[1],
           g_robot_home_pose.joint_deg[2]);
    AppHardware_SetModeIndicator(true, false);
    AppHardware_StartStepTimer();
    return true;
}

void App_Init(void)
{
    PlatformTime_Init();
    AppHardware_Init();
    AppHardware_SendText("# System Boot Up OK!\r\n");

    PlatformDelay_Ms(100U);
    printf("# System Boot Up OK! Gcode Mode\r\n");

    AppHardware_ConfigureMotionDrivers();
    SafetyService_Init();
    App_StartHoming(false);
}

void App_RunOnce(void)
{
    static bool fault_reported = false;

    if (s_boot_state == APP_BOOT_HOMING) {
        /* Commands received while homing must not execute after recovery. */
        AppHardware_DiscardRx();
        Svc_Homing_Step();
        if (Svc_Homing_GetState() == SVC_HOMING_COMPLETE) {
            if (App_FinalizeAfterHoming()) {
                s_boot_state = APP_BOOT_READY;
                s_mode_switch_requested = false;
                if (s_homing_is_recovery) printf("M999OK\r\n");
                else printf("# App runtime initialized.\r\n");
            } else {
                AppHardware_EnableJoints(false);
                s_boot_state = APP_BOOT_FAILED;
            }
        } else if (Svc_Homing_GetState() == SVC_HOMING_FAILED) {
            printf("error: %shoming failed; motion disabled\r\n",
                   s_homing_is_recovery ? "M999 " : "");
            AppHardware_EnableJoints(false);
            s_boot_state = APP_BOOT_FAILED;
        }
        return;
    }
    if (s_boot_state == APP_BOOT_FAILED) return;

    MotionService_ServiceSafety();
    App_GCodeExec_Service();
    if (AppHardware_TakeTxOverflow()) AppHardware_SendText("# WARN: UART TX queue overflow; log dropped\r\n");
    if (SafetyService_HasFault()) {
        AppHardware_SetModeIndicator(false, true);
        if (!fault_reported) {
            printf("error: motion safety stop (%s); re-home required\r\n", App_MotionFaultText(MotionService_GetFaultReason()));
            fault_reported = true;
        }
        App_ModeSwitchTask();
        Svc_Gripper_IdleStop();
        App_GCodeTask();
        return;
    }

    fault_reported = false;
    App_ModeSwitchTask();
    App_Teleop_Task();
    SystemMode_t mode = App_Teleop_GetMode();
    App_UpdateModeIndicator(mode);
    if (mode == SYS_MODE_PS2) AppHardware_DiscardRx();
    if (mode == SYS_MODE_GCODE) {
        App_EncoderReportTask();
        App_ClosedLoopTask();
    } else {
        App_EncoderReadTask();
    }
    Svc_Gripper_IdleStop();
    if (mode == SYS_MODE_GCODE) App_GCodeTask();
}
