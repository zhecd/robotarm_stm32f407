/**
 * @file app_runtime.c
 * @brief 应用编排：启动、回零、恢复、模式切换与周期任务调度。
 * @details 正式固件保留启动自动回零；ROBOTARM_DIAGNOSTIC_MODE=1 选择只读入口。
 */
#include "app_init.h"

#include "app_hardware_adapter.h"
#include "app/app_runtime_events.h"
#include "service/svc_gripper.h"
#include "service/svc_homing.h"
#include "motion_service.h"
#include "platform_delay.h"
#include "platform_time.h"
#include "platform_critical.h"
#include "safety_service.h"
#include "state_service.h"
#include "app/app_calibration.h"
#include "app/app_gcode_exec.h"
#include "app/app_serial_protocol.h"
#include "app/app_diagnostics.h"
#include "app/app_waveform.h"
#include "command_service.h"
#include "app/app_teleop.h"
#include "common/robot_home_pose.h"
#include "robot_config.h"
#include "limit_irq_guard.h"
#if ROBOTARM_SEQUENTIAL_HOMING
#include "service/svc_homing_sequence.h"
#endif

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

static volatile bool s_mode_switch_requested = false;


typedef enum {
    APP_BOOT_HOMING = 0,
    APP_BOOT_READY,
    APP_BOOT_FAILED,
    APP_BOOT_CHECKING,
    APP_BOOT_OUTPUT_LOCKED
} AppBootState_t;

static AppBootState_t s_boot_state = APP_BOOT_HOMING;
static bool s_homing_is_recovery = false;
static bool s_gripper_initialized;
static uint32_t s_check_started_ms, s_check_last_poll_ms;
static bool s_check_first_poll;
static bool s_irq_fault_reported;
#if ROBOTARM_SEQUENTIAL_HOMING
static bool s_calibration_hold;
static uint32_t s_calibration_hold_ms;
#endif

static void App_BeginStartup(bool recovery);
static void App_ProcessMotionFaults(void);
static bool App_ServiceIrqOverload(void);

#if ROBOTARM_STARTUP_TRIAL_MODE
/** 专用试验在首次RunOnce前等待人工许可；等待时长不应消耗编码器检查期限。
 * 只重置尚未开始的检查计时，不清故障、不重配驱动、不产生输出。 */
bool AppTrial_ReleasePreparedStartup(void)
{
    if(s_boot_state!=APP_BOOT_CHECKING || s_gripper_initialized ||
       SafetyService_HasFault() || LimitIrqGuard_GetLatchMask()!=0U)return false;
    s_check_started_ms=PlatformTime_NowMs();s_check_first_poll=true;
    return true;
}
#endif

void AppRuntime_RequestModeSwitch(void)
{
    s_mode_switch_requested = true;
}

static void App_EncoderReadTask(void)
{
    static uint32_t last_tick = 0U;
    uint32_t now = PlatformTime_NowMs();
    if ((now - last_tick) < CL_UPDATE_MS) return;
    last_tick = now;

    for (int i = 0; i < (int)MOTION_AXIS_COUNT; i++) {
        if (MotionService_IsClosedLoopAxisEnabled(i)) {
            float angle;
            MotionService_GetClosedLoopAxisAngle(i, &angle);
            if (MotionService_HasFault()) break;
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
    App_ProcessMotionFaults();
}

static void App_ModeSwitchTask(void)
{
    static uint32_t last_switch_ms = 0U;
    if (!s_mode_switch_requested) return;
    s_mode_switch_requested = false;
#if defined(ROBOTARM_JOINT_RETURN_TEST) && ROBOTARM_JOINT_RETURN_TEST
    if (AppSerial_JointTrialActive()) return;
#endif

    uint32_t now = PlatformTime_NowMs();
    if ((now - last_switch_ms) < 200U) return;
    last_switch_ms = now;
    /* ISR刚排空最后一帧时，闭环可能还有待生成段或完成收尾。
     * 此时切入PS2会停用闭环任务，留下RECOVERY状态，使手柄一直被拒绝。 */
    if (!MotionService_IsIdle() || App_GCodeExec_IsMotionPending() ||
        MotionService_IsClosedLoopRecoveryActive()) {
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
    case MOTION_FAULT_CONTROL_DIVERGENCE: return "closed-loop correction divergence";
    case MOTION_FAULT_CORRECTION_RANGE: return "correction error exceeds joint recovery window";
    case MOTION_FAULT_CORRECTION_TIMEOUT: return "closed-loop correction recovery timeout";
    case MOTION_FAULT_CORRECTION_STALLED: return "closed-loop correction made no progress";
    case MOTION_FAULT_HOST_STOP: return "host requested stop";
    case MOTION_FAULT_LIMIT_IRQ_OVERLOAD: return "limit IRQ overload";
    case MOTION_FAULT_INPUT_LOST: return "PS2 input lost";
    default: return "unspecified";
    }
}

/* Motion owns immediate stopping.  App translates its delivered fault event
 * into the safety state used by the command and recovery layers. */
static void App_ProcessMotionFaults(void)
{
    MotionFaultReason_t reason;
    while (MotionService_TakeFaultEvent(&reason)) {
        switch (reason) {
        case MOTION_FAULT_LIMIT_SWITCH:
            SafetyService_ReportLimitSwitch();
            break;
        case MOTION_FAULT_ENCODER:
            SafetyService_ReportEncoderFailure();
            break;
        case MOTION_FAULT_SOFT_LIMIT:
            SafetyService_ReportSoftLimit();
            break;
        case MOTION_FAULT_QUEUE_TIMEOUT:
            SafetyService_ReportQueueTimeout();
            break;
        case MOTION_FAULT_CONTROL_DIVERGENCE:
        case MOTION_FAULT_CORRECTION_RANGE:
        case MOTION_FAULT_CORRECTION_TIMEOUT:
        case MOTION_FAULT_CORRECTION_STALLED:
            SafetyService_ReportControlDivergence();
            break;
        case MOTION_FAULT_HOST_STOP:
            SafetyService_ReportHostStop();
            break;
        case MOTION_FAULT_LIMIT_IRQ_OVERLOAD:
            SafetyService_ReportLimitIrqOverload();
            break;
        case MOTION_FAULT_INPUT_LOST:
            SafetyService_ReportInputLost();
            break;
        default:
            SafetyService_ReportInternalFailure();
            break;
        }
        CommandService_AbortPending();
    }
}

/** ISR停止硬件，主循环取消回零和规划；过载锁存不允许在线清除。 */
static bool App_ServiceIrqOverload(void)
{
    if (LimitIrqGuard_GetLatchMask()==0U) {
#if ROBOTARM_SEQUENTIAL_HOMING
        /* 回零控制器首因另存M126；不把接触/释放失败误标为IRQ过载。 */
        if(s_boot_state==APP_BOOT_HOMING)(void)Svc_HomingSequence_ValidateHandoff();
        if(Svc_HomingSequence_HasFailure()){
            AppHardware_StopActuators();CommandService_AbortPending();
            SafetyService_ReportInternalFailure();s_boot_state=APP_BOOT_FAILED;
            AppSerial_SetMode(APP_SERIAL_FAILED);return true;
        }
#endif
        return false;
    }
    AppHardware_StopActuators();
    Svc_Homing_Cancel();
    MotionService_AbortForFault(MOTION_FAULT_LIMIT_IRQ_OVERLOAD);
    SafetyService_ReportLimitIrqOverload();
    CommandService_AbortPending();
    s_boot_state=APP_BOOT_FAILED;
    AppSerial_SetMode(APP_SERIAL_FAILED);
    if (!s_irq_fault_reported) {
        printf("error: limit IRQ overload; restart required after inspection\r\n");
        s_irq_fault_reported=true;
    }
    return true;
}

/** 仅申请启动；旧故障保持锁存，直到回零与标定成功才清除。 */
static void App_BeginStartup(bool recovery)
{
#if ROBOTARM_SEQUENTIAL_HOMING
    s_calibration_hold=false;
#endif
    CommandService_AbortPending();
#if ROBOTARM_SEQUENTIAL_HOMING
    /* 只有明确M999可重建非IRQ失败的本轮滤波器；不清原安全故障或IRQ锁存。 */
    if(recovery)(void)Svc_HomingSequence_PrepareStartup(true);
#endif
    if (App_ServiceIrqOverload()) return;
#if ROBOTARM_SEQUENTIAL_HOMING
    if(!Svc_HomingSequence_PrepareStartup(false)){
        (void)App_ServiceIrqOverload();return;
    }
#else
    Svc_Homing_Cancel();
#endif
    AppSerial_SetMode(APP_SERIAL_CHECKING);
    MotionService_SetLimitMonitoring(false);
    AppHardware_EnableJoints(false);
    s_homing_is_recovery = recovery;
    /* 每次启动/显式M999均重新确认配置；不能用HAL发送完成代替芯片接受。
     * 失败不自动重试，不初始化夹爪、不使能关节，查询仍由主循环处理。 */
    uint8_t failed_node;
    bool drivers_ready=AppHardware_PrepareMotionDrivers(&failed_node);
    if (App_ServiceIrqOverload()) return;
    if (!drivers_ready) {
        SafetyService_ReportDriverConfigFailure();
        s_boot_state=APP_BOOT_FAILED;
        AppSerial_SetMode(APP_SERIAL_FAILED);
        printf("error: driver configuration unconfirmed; node=%u; use M999 to retry\r\n",failed_node);
        return;
    }
    s_check_started_ms = PlatformTime_NowMs();
    s_check_first_poll = true;
    s_boot_state = APP_BOOT_CHECKING;
}

/** 检查阶段不置零、不启用反馈控制；每轮都重新读取三轴，不累计过期的成功位。 */
static void App_CheckEncoderReadiness(void)
{
    uint32_t now = PlatformTime_NowMs();
    uint8_t valid_mask = 0U;
    if ((uint32_t)(now - s_check_started_ms) < STARTUP_ENCODER_TIMEOUT_MS) {
        if (!s_check_first_poll &&
            (uint32_t)(now - s_check_last_poll_ms) < STARTUP_ENCODER_POLL_MS) return;
        s_check_first_poll = false;
        s_check_last_poll_ms = now;
        for (uint8_t axis=0; axis<MOTION_AXIS_COUNT; ++axis) {
            float angle;
            bool read_ok = AppHardware_ReadMotorAngle(axis, &angle);
            /* I2C有限阻塞期间ISR仍可锁存过载。先处理该锁存，再判断反馈或期限；
             * 否则失败读取会延迟处理，跨期限返回会把IRQ首因误记为编码器超时。
             * 检出后本轮不再读取后续轴，也不初始化夹爪或推进回零。 */
            if (App_ServiceIrqOverload()) return;
            if (read_ok && isfinite(angle))
                valid_mask |= (uint8_t)(1U << axis);
        }
        /* HAL读取仍有有限阻塞，必须在读取后再次检查期限，不能接受逾期成功。 */
        now = PlatformTime_NowMs();
        if ((uint32_t)(now - s_check_started_ms) < STARTUP_ENCODER_TIMEOUT_MS &&
            valid_mask != ((1U << MOTION_AXIS_COUNT) - 1U)) return;
    }
    /* 包含本轮因期限已到而没有进入读取循环的情况。 */
    if (App_ServiceIrqOverload()) return;
    if ((uint32_t)(now - s_check_started_ms) >= STARTUP_ENCODER_TIMEOUT_MS) {
        SafetyService_ReportEncoderFailure();
        AppHardware_EnableJoints(false);
        s_boot_state = APP_BOOT_FAILED;
        AppSerial_SetMode(APP_SERIAL_FAILED);
        printf("error: startup encoder timeout (valid mask %u); motion disabled\r\n",
               (unsigned)valid_mask);
        return;
    }
    /* 保留正常固件自动夹爪初始化和回零功能，但只在反馈就绪后执行。 */
    if (App_ServiceIrqOverload()) return;
#if ROBOTARM_SEQUENTIAL_HOMING
    /* 尚在上电触点稳定确认时继续查询；未就绪不初始化夹爪、不使能关节。 */
    if(!Svc_HomingSequence_BootReady())return;
#endif
#if ROBOTARM_STARTUP_AUDIT_MODE
    /* 真实配置/编码器检查已完成；审计在夹爪初始化与EN使能之前停下。
     * 不模拟回零、标定或清故障，旧故障仍锁存，M112及查询继续运行。 */
    AppHardware_EnableJoints(false);
    s_boot_state=APP_BOOT_OUTPUT_LOCKED;
    AppSerial_SetMode(APP_SERIAL_OUTPUT_LOCKED);
    printf("# Startup checks passed; outputs locked; homing not performed\r\n");
    return;
#endif
    if (!s_gripper_initialized) {
        if (!AppHardware_TryInitGripper()) {
            /* 只有夹爪端口真正受理后才能启用关节、进入回零。
             * 先处理同期IRQ首因；普通PWM失败使用既有INTERNAL故障。
             * 保留未初始化标记，不自动重试；显式M999可重新检查。
             */
            if (App_ServiceIrqOverload()) return;
            SafetyService_ReportInternalFailure();
            AppHardware_StopActuators();
            s_boot_state = APP_BOOT_FAILED;
            AppSerial_SetMode(APP_SERIAL_FAILED);
            printf("error: startup gripper initialization failed; motion disabled\r\n");
            return;
        }
        /* 端口恢复中断后，刚到达的过载IRQ可能已锁存并关闭输出。
         * 必须先映射该首因，不能继续请求EN或把后续Start拒绝误记为INTERNAL。 */
        if (App_ServiceIrqOverload()) return;
        s_gripper_initialized = true;
    }
    AppSerial_SetMode(APP_SERIAL_HOMING);
#if !ROBOTARM_SEQUENTIAL_HOMING
    AppHardware_EnableJoints(true);
#endif
    if (App_ServiceIrqOverload()) return;
    if (!Svc_Homing_Start()) {
        /* 覆盖上次检查与服务入口之间的IRQ窗口，保留过载首因。 */
        if (App_ServiceIrqOverload()) return;
        SafetyService_ReportInternalFailure();
        AppHardware_EnableJoints(false);
        s_boot_state = APP_BOOT_FAILED;
        AppSerial_SetMode(APP_SERIAL_FAILED);
        return;
    }
    s_boot_state = APP_BOOT_HOMING;
    (void)App_ServiceIrqOverload();
}

/* 仅在回零状态机完成后执行标定与参考提交；完成STEP不等于已允许正常运动。 */
static bool App_FinalizeAfterHoming(void)
{
    if (App_ServiceIrqOverload()) return false;
    MotionService_Init();
    if (App_GCodeExec_Init(g_robot_home_pose.x_mm, g_robot_home_pose.y_mm,
                           g_robot_home_pose.z_mm) != ERR_OK) {
        printf("error: initial pose is unreachable; motion disabled\r\n");
        return false;
    }

    App_Teleop_Init();
    MotionService_InitClosedLoop();
    StateService_Init();
    bool calibrated = App_Calibration_Execute();
    /* 标定含有限I2C读取；失败与同期过载同时返回时仍应保留IRQ首因。 */
    if (App_ServiceIrqOverload()) return false;
    if (!calibrated) {
        printf("error: encoder calibration failed; motion disabled\r\n");
        return false;
    }

    /* 只把许可提交放在短临界区：检查锁存、参考/故障状态更新构成同一事务。
     * 标定、日志和硬件端口均在区外，不能为了提交状态长期屏蔽中断。
     * 区内新到达的IRQ将在恢复中断后交付，必须立即复查，不能继续报成功。 */
    PlatformCriticalState_t key = PlatformCritical_Enter();
    if (LimitIrqGuard_GetLatchMask()!=0U) {
        PlatformCritical_Exit(key);
        (void)App_ServiceIrqOverload();
        return false;
    }
#if ROBOTARM_SEQUENTIAL_HOMING
    if(!Svc_HomingSequence_CommitReady()){
        PlatformCritical_Exit(key);return false;
    }
#endif
    SafetyService_MarkHomed();
    SafetyService_ClearAfterSuccessfulHoming();
    MotionService_ClearFault();
    MotionService_SetLimitMonitoring(true);
    PlatformCritical_Exit(key);
    if (App_ServiceIrqOverload()) return false;
#if ROBOTARM_SEQUENTIAL_HOMING
    /* 逐轴服务完成时已禁用EN；正式READY交接才恢复原固件的保持使能。 */
    AppHardware_EnableJoints(true);
    if(App_ServiceIrqOverload())return false;
#endif
    printf("# [HomePose] XYZ=%.1f,%.1f,%.1f J=%.1f,%.1f,%.1f\r\n",
           g_robot_home_pose.x_mm, g_robot_home_pose.y_mm, g_robot_home_pose.z_mm,
           g_robot_home_pose.joint_deg[0], g_robot_home_pose.joint_deg[1],
           g_robot_home_pose.joint_deg[2]);
    /* M999可能沿用PS2控制模式，显示必须取真实模式，不能强写成串口常亮。 */
    AppHardware_SetModeIndicator(App_Teleop_GetMode()==SYS_MODE_GCODE,
                                 App_Teleop_GetMode()==SYS_MODE_PS2);
    AppHardware_StartStepTimer();
    /* 端口恢复中断后也可能已有新锁存；本轮不能发布READY或M999OK。 */
    return !App_ServiceIrqOverload();
}

void App_Init(void)
{
#if ROBOTARM_WAVEFORM_MODE
    AppWaveform_Init();
    return;
#endif
#if ROBOTARM_DIAGNOSTIC_MODE
    AppDiagnostics_Init();
    return;
#endif
    AppSerial_Init(APP_SERIAL_CHECKING);
    PlatformTime_Init();
    s_gripper_initialized = false;
    s_irq_fault_reported = false;
    AppHardware_Init();
    AppHardware_SendText("# System Boot Up OK!\r\n");

    PlatformDelay_Ms(100U);
    printf("# System Boot Up OK! Gcode Mode\r\n");

    SafetyService_Init();
    App_BeginStartup(false);
}

void App_RunOnce(void)
{
#if !ROBOTARM_SEQUENTIAL_HOMING
    /* 无1ms回零钩子的独立诊断入口也持续显示，不受READY/FAILED提前返回影响。 */
    AppHardware_UpdateLimitIndicators();
#endif
#if ROBOTARM_WAVEFORM_MODE
    AppWaveform_RunOnce();
    return;
#endif
#if ROBOTARM_DIAGNOSTIC_MODE
    AppDiagnostics_RunOnce();
    return;
#endif
    static bool fault_reported = false;
    (void)App_ServiceIrqOverload();
    if (s_boot_state == APP_BOOT_READY) {
        /* 先同步已确认的故障，再处理新请求/完成回复；此处不生成补偿轨迹。 */
        MotionService_ServiceSafety();
        App_ProcessMotionFaults();
        if (!SafetyService_HasFault() && App_Teleop_GetMode()==SYS_MODE_PS2) {
            App_EncoderReadTask();
            App_ProcessMotionFaults();
        }
    }
    /* 通信独立于回零与运动等待推进，M112/状态查询不会被 M400 阻塞。 */
    AppSerial_Service();
    if (AppSerial_TakeStopRequest() &&
        (s_boot_state == APP_BOOT_CHECKING || s_boot_state == APP_BOOT_HOMING ||
         s_boot_state == APP_BOOT_OUTPUT_LOCKED)) {
        /* M999沿用旧故障，重复M112未必改变Safety generation，必须消费独立停止事件。 */
        Svc_Homing_Cancel();
        AppHardware_EnableJoints(false);
        s_boot_state = APP_BOOT_FAILED;
        AppSerial_SetMode(APP_SERIAL_FAILED);
    }
    if (AppSerial_TakeRecoveryRequest()) App_BeginStartup(true);
    if (App_ServiceIrqOverload()) return;

    if (s_boot_state == APP_BOOT_CHECKING) {
        App_CheckEncoderReadiness();
        return;
    }

    if (s_boot_state == APP_BOOT_HOMING) {
        /* 初始化夹爪后回零可能持续数十秒；保持计时不能等READY才推进。
         * 这里仅停止已到期PWM，不发新目标，也不延长原600ms保持窗口。 */
        Svc_Gripper_IdleStop();
        /* Commands received while homing must not execute after recovery. */
        if (SafetyService_HasFault() && !s_homing_is_recovery) {
            Svc_Homing_Cancel();
            AppHardware_EnableJoints(false);
            s_boot_state = APP_BOOT_FAILED;
            AppSerial_SetMode(APP_SERIAL_FAILED);
            return;
        }
        Svc_Homing_Step(); /* 服务内部保护短输出片段，状态日志在临界区外。 */
        if (App_ServiceIrqOverload()) return;
        if (Svc_Homing_GetState() == SVC_HOMING_COMPLETE) {
#if ROBOTARM_SEQUENTIAL_HOMING
            /* 退回结束曾禁用EN，不能在松力状态标定后又突然恢复保持力。
             * 先只保持使能100ms；期间继续协议、IRQ保护及心跳，不发STEP。 */
            if(!s_calibration_hold){
                if(!Svc_HomingSequence_HoldForCalibration()){
                    Svc_Homing_Cancel();(void)App_ServiceIrqOverload();return;
                }
                s_calibration_hold=true;s_calibration_hold_ms=PlatformTime_NowMs();return;
            }
            if((uint32_t)(PlatformTime_NowMs()-s_calibration_hold_ms)<100U)return;
#endif
            if (App_FinalizeAfterHoming()) {
                /* 丢弃回零期间尚未消费的输入，禁止旧移动命令在 READY 后补执行。 */
                AppHardware_DiscardRx();
                if (App_ServiceIrqOverload()) return;
                s_boot_state = APP_BOOT_READY;
                AppSerial_SetMode(APP_SERIAL_READY);
                if (App_ServiceIrqOverload()) return;
                s_mode_switch_requested = false;
                if (s_homing_is_recovery) printf("M999OK\r\n");
                else printf("# App runtime initialized.\r\n");
            } else {
                if (App_ServiceIrqOverload()) return;
#if ROBOTARM_SEQUENTIAL_HOMING
                Svc_Homing_Cancel(); /* 失败后SysTick不得重新保持EN。 */
#endif
                AppHardware_EnableJoints(false);
                s_boot_state = APP_BOOT_FAILED;
                SafetyService_ReportInternalFailure();
                AppSerial_SetMode(APP_SERIAL_FAILED);
            }
        } else if (Svc_Homing_GetState() == SVC_HOMING_FAILED) {
            printf("error: %shoming failed; motion disabled\r\n",
                   s_homing_is_recovery ? "M999 " : "");
            AppHardware_EnableJoints(false);
            s_boot_state = APP_BOOT_FAILED;
            SafetyService_ReportInternalFailure();
            AppSerial_SetMode(APP_SERIAL_FAILED);
        }
        return;
    }
    /* 审计动作边界仍处理前面的协议/停止，但不能落入READY周期任务。 */
    if (s_boot_state == APP_BOOT_FAILED || s_boot_state == APP_BOOT_OUTPUT_LOCKED) return;

    MotionService_ServiceSafety();
    App_ProcessMotionFaults();
    App_GCodeExec_Service();
    ErrorCode_t queued_result;
    while (App_GCodeExec_TakeQueuedMoveResult(&queued_result)) {
        if (queued_result == ERR_OK) {
            MotionService_SyncClosedLoopTarget();
        } else {
            printf("error: queued command rejected (%d); queue cleared\r\n",
                   (int)queued_result);
        }
    }
    if (AppHardware_TakeTxOverflow()) AppHardware_SendText("# WARN: UART TX queue overflow; log dropped\r\n");
    if (SafetyService_HasFault()) {
        /* 故障不改变模式；不可用“PS2灯态”冒充故障灯，详细首因由串口报告。 */
        AppHardware_SetModeIndicator(App_Teleop_GetMode()==SYS_MODE_GCODE,
                                     App_Teleop_GetMode()==SYS_MODE_PS2);
        if (!fault_reported) {
            printf("error: motion safety stop (%s); re-home required\r\n", App_MotionFaultText(MotionService_GetFaultReason()));
            fault_reported = true;
        }
        App_ModeSwitchTask();
        Svc_Gripper_IdleStop();
        return;
    }

    fault_reported = false;
    App_ModeSwitchTask();
    /* MODE刚切入PS2时，本轮前段尚未执行PS2采样，先检查反馈再处理按键动作。 */
    if (App_Teleop_GetMode()==SYS_MODE_PS2) {
        App_EncoderReadTask();
        MotionService_ServiceSafety();
        App_ProcessMotionFaults();
        if (SafetyService_HasFault()) { Svc_Gripper_IdleStop();return; }
    }
    App_Teleop_Task();
    if (App_ServiceIrqOverload()) return;
    App_ProcessMotionFaults();
    if (SafetyService_HasFault()) { Svc_Gripper_IdleStop();return; }
    SystemMode_t mode = App_Teleop_GetMode();
    App_UpdateModeIndicator(mode);
    /* PS2 模式保留状态查询/停止接口；运动命令由 AppSerial 明确拒绝。 */
    if (mode == SYS_MODE_GCODE) {
        App_EncoderReportTask();
        App_ClosedLoopTask();
    }
    Svc_Gripper_IdleStop();

}
