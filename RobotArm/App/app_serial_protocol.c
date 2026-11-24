/**
 * @file app_serial_protocol.c
 * @brief 串口协议会话：命令解析、接收确认、完成等待、查询和停止。
 * @details G-code 的 ok/queued 表示接收。M400 等待运动队列清空和电机侧闭环到位。
 */
#include "app/app_serial_protocol.h"
#include "app/app_gcode_parser.h"
#include "app/app_gcode_exec.h"
#include "app/app_teleop.h"
#include "app/app_waveform.h"
#include "app/app_diagnostics.h"
#include "limit_observation_service.h"
#include "platform_time.h"
#include "waveform_service.h"
#include "app_hardware_adapter.h"
#include "command_service.h"
#include "motion_service.h"
#include "safety_service.h"
#include "state_service.h"
#include "service/svc_homing.h"
#include "service/svc_gripper.h"
#include "limit_irq_guard.h"
#include "robot_config.h"
#if ROBOTARM_SEQUENTIAL_HOMING
#include "service/svc_homing_sequence.h"
#endif
#include <stdarg.h>
#if defined(ROBOTARM_JOINT_RETURN_TEST) && ROBOTARM_JOINT_RETURN_TEST
#include <math.h>
#endif
#include <stdio.h>
#include <string.h>

static AppSerialMode_t s_mode;
static bool s_wait_move;
static bool s_wait_plan;
static bool s_recovery_request;
static bool s_stop_request;
#if defined(ROBOTARM_JOINT_RETURN_TEST) && ROBOTARM_JOINT_RETURN_TEST
/* 只供本次 A/B：来自 Z170 的 M411 整数微步目标，方向遵循生产 DDA。 */
enum { JOINT_TRIAL_M2 = -341, JOINT_TRIAL_M3 = 1265,
       JOINT_TRIAL_FRAMES = 50, JOINT_TRIAL_FRAME_MS = 62 };
_Static_assert(JOINT_TRIAL_FRAMES < RING_BUFFER_SIZE, "Joint test must fit the idle queue");
_Static_assert((int) STEPS_PER_REV == 3200, "Fixed joint test requires 3200 motor steps per rev");
typedef enum { JOINT_TRIAL_ARMED, JOINT_TRIAL_LOW, JOINT_TRIAL_RETURNING, JOINT_TRIAL_DONE } JointTrialPhase_t;
static JointTrialPhase_t s_joint_trial;

bool AppSerial_JointTrialActive(void)
{
    return s_joint_trial == JOINT_TRIAL_LOW || s_joint_trial == JOINT_TRIAL_RETURNING;
}
#endif


/* 一次提交完整回复，避免逐字符 printf 在队列满时留下半条协议行。 */
static void Reply(const char *format, ...)
{
    /* DIAG3含两组三轴32位计数和限流锁存位图，最大值回复也必须完整容纳。 */
    char text[384];
    va_list args;
    va_start(args, format);
    int size = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (size > 0 && (size_t)size < sizeof(text)) AppHardware_SendText(text);
    else AppHardware_SendText("error: response too long\r\n");
}

void AppSerial_SetMode(AppSerialMode_t mode)
{
#if defined(ROBOTARM_JOINT_RETURN_TEST) && ROBOTARM_JOINT_RETURN_TEST
    if (mode != APP_SERIAL_READY && AppSerial_JointTrialActive()) s_joint_trial = JOINT_TRIAL_DONE;
#endif
    s_mode = mode;
    s_wait_move = s_wait_plan = false;
}
void AppSerial_Init(AppSerialMode_t mode)
{
#if defined(ROBOTARM_JOINT_RETURN_TEST) && ROBOTARM_JOINT_RETURN_TEST
    s_joint_trial = JOINT_TRIAL_ARMED;
#endif
    AppSerial_SetMode(mode);
    s_recovery_request = false;
    s_stop_request = false;
}
bool AppSerial_TakeStopRequest(void)
{
    bool request = s_stop_request;
    s_stop_request = false;
    return request;
}
bool AppSerial_TakeRecoveryRequest(void)
{
    bool request = s_recovery_request;
    s_recovery_request = false;
    return request;
}
static bool MotionCompleted(void)
{
    return MotionService_IsIdle() && !App_GCodeExec_IsMotionPending() &&
           MotionService_HasClosedLoopSettled();
}
static void PollPendingReply(void)
{
    if ((s_wait_plan || s_wait_move) && SafetyService_HasFault()) {
        Reply(s_wait_move ? "error: M400 aborted by safety fault\r\n" :
                            "error: command aborted by safety fault\r\n");
        s_wait_plan = s_wait_move = false;
        return;
    }
    if (s_wait_plan) {
        ErrorCode_t result;
        if (App_GCodeExec_TakeMoveResult(&result)) {
            s_wait_plan = false;
            if (result == ERR_OK) {
                MotionService_SyncClosedLoopTarget();
                Reply("ok\r\n");
            } else Reply("error: command rejected (%d)\r\n", (int)result);
        }
    }
    if (s_wait_move && MotionCompleted()) {
        s_wait_move = false;
        Reply("ok\r\n");
    }
}
static void ReportStatus(void)
{
    if (s_mode==APP_SERIAL_DIAGNOSTIC) {
        LimitObservationStatus_t observed;
        LimitObservation_GetStatus(&observed);
        Reply("M408 STATE:DIAGNOSTIC HOMED:0 ALLOWED:0 EN:DISABLED UPTIME_MS:%lu SAMPLE_MS:%lu ENCODER_POLL:%u RAW_MASK:%u STABLE_MASK:%u VALID_MASK:%u EDGE:%lu,%lu,%lu PRESS:%lu,%lu,%lu IRQ_LATCH_MASK:%u\r\n",
              (unsigned long)PlatformTime_NowMs(),(unsigned long)observed.sample_ms,
              AppDiagnostics_EncoderSamplingEnabled()?1U:0U,
              (unsigned)observed.raw_mask,(unsigned)observed.stable_mask,(unsigned)observed.valid_mask,
              (unsigned long)observed.edges[0],(unsigned long)observed.edges[1],(unsigned long)observed.edges[2],
              (unsigned long)observed.presses[0],(unsigned long)observed.presses[1],(unsigned long)observed.presses[2],
              (unsigned)observed.irq_latch_mask);
        return;
    }
    if (s_mode==APP_SERIAL_WAVEFORM) {
        WaveformStatus_t wave;
        WaveformService_GetStatus(&wave);
        Reply("M408 STATE:WAVEFORM HOMED:0 ALLOWED:0 EN:DISABLED WAVE_STATE:%u NEXT_ID:%lu AXIS:%u EMITTED:%u LIMIT_MASK:%u LIMIT_RAW:%u LIMIT_SOURCE:%u EDGE:%lu,%lu,%lu IRQ_LATCH_MASK:%u ENCODER_POLL_MASK:%u\r\n",
              (unsigned)wave.state,(unsigned long)wave.next_id,
              (unsigned)wave.axis+1U,(unsigned)wave.emitted,
              (unsigned)wave.limit_mask,(unsigned)wave.limit_raw_mask,(unsigned)wave.limit_source,
              (unsigned long)wave.limit_edges[0],(unsigned long)wave.limit_edges[1],
              (unsigned long)wave.limit_edges[2],(unsigned)LimitIrqGuard_GetLatchMask(),
              (unsigned)AppWaveform_EncoderPollMask());
        return;
    }
    SafetyServiceStatus_t safety;
    SafetyService_GetStatus(&safety);
    static const char *names[] = {"HOMING", "READY", "FAILED", "DIAGNOSTIC", "WAVEFORM", "CHECKING", "OUTPUT_LOCKED"};
    Reply("M408 STATE:%s HOMED:%u ALLOWED:%u MODE:%s FAULT:%u MOTION_FAULT:%u QUEUE:%u PENDING:%u RECOVERY:%u IRQ_LATCH_MASK:%u\r\n",
          names[s_mode], safety.homed ? 1U:0U, safety.motion_allowed ? 1U:0U,
          App_Teleop_GetMode()==SYS_MODE_GCODE ? "GCODE":"PS2", (unsigned)safety.fault,
          (unsigned)MotionService_GetFaultReason(), (unsigned)CommandService_GetQueuedMoveCount(),
          App_GCodeExec_IsMotionPending() ? 1U:0U,
          MotionService_IsClosedLoopRecoveryActive() ? 1U:0U,(unsigned)LimitIrqGuard_GetLatchMask());
}
static void ReportPosition(void)
{
    StateServiceStatus_t state;
    StateService_GetStatus(&state);
    SafetyServiceStatus_t safety;
    SafetyService_GetStatus(&safety);
    if (s_mode == APP_SERIAL_DIAGNOSTIC || s_mode == APP_SERIAL_WAVEFORM) {
        Reply("M114 RAWENC:%.2f,%.2f,%.2f VALID:%u,%u,%u\r\n",
              state.motor_angle_deg[0],state.motor_angle_deg[1],state.motor_angle_deg[2],
              state.encoder_valid[0]?1U:0U,state.encoder_valid[1]?1U:0U,state.encoder_valid[2]?1U:0U);
    } else if (s_mode == APP_SERIAL_READY && safety.homed && !safety.fault_latched) {
        float x,y,z;
        App_GCodeExec_GetPlannedPosition(&x,&y,&z);
        Reply("M114 PLAN X:%.2f Y:%.2f Z:%.2f J:%.2f,%.2f,%.2f ENC:%.2f,%.2f,%.2f\r\n",
              x,y,z,state.joint_angle_deg[0],state.joint_angle_deg[1],state.joint_angle_deg[2],
              state.motor_angle_deg[0],state.motor_angle_deg[1],state.motor_angle_deg[2]);
    } else Reply("error: position unavailable before successful homing\r\n");
}

static void ReportFeedback(void)
{
    /* 同一份快照包含角度和逐轴时间，避免拼接两条M114/M408时混淆样本。
     * PRESENT为曾有成功样本，VALID为最后读取结果，FRESH再检查逐轴年龄。 */
    StateServiceStatus_t s;StateService_GetStatus(&s);
    uint32_t now=PlatformTime_NowMs(),age[3],read_ms[3];
    uint8_t present=0U,valid=0U;
    for(uint8_t axis=0;axis<3;axis++) {
        if(s.sample_present[axis])present|=(uint8_t)(1U<<axis);
        if(s.encoder_valid[axis])valid|=(uint8_t)(1U<<axis);
        age[axis]=s.sample_present[axis]?(uint32_t)(now-s.sample_started_ms[axis]):UINT32_MAX;
        read_ms[axis]=s.sample_present[axis]?(uint32_t)(s.sample_finished_ms[axis]-s.sample_started_ms[axis]):0U;
    }
    /* READY是协议阶段，homed才是当前参考的有效性。断能撤销或故障后，
     * 原始编码器仍可读，但旧JOINT/PLAN不能再作为有效空间坐标使用。 */
    SafetyServiceStatus_t safety;SafetyService_GetStatus(&safety);
    const char *frame=s_mode==APP_SERIAL_READY && safety.homed && !safety.fault_latched?"JOINT":
        (s_mode==APP_SERIAL_DIAGNOSTIC || s_mode==APP_SERIAL_WAVEFORM)?"RAW":"UNAVAILABLE";
    Reply("M409 GENERATION:%lu NOW_MS:%lu FRAME:%s PRESENT_MASK:%u VALID_MASK:%u FRESH_MASK:%u MAX_AGE_MS:%lu AGE_MS:%lu,%lu,%lu READ_MS:%lu,%lu,%lu MOTOR:%.6g,%.6g,%.6g JOINT:%.6g,%.6g,%.6g\r\n",
        (unsigned long)s.generation,(unsigned long)now,frame,present,valid,
        StateService_FreshMask(&s,now,FEEDBACK_QUERY_MAX_AGE_MS),(unsigned long)FEEDBACK_QUERY_MAX_AGE_MS,
        (unsigned long)age[0],(unsigned long)age[1],(unsigned long)age[2],
        (unsigned long)read_ms[0],(unsigned long)read_ms[1],(unsigned long)read_ms[2],
        (double)s.motor_angle_deg[0],(double)s.motor_angle_deg[1],(double)s.motor_angle_deg[2],
        (double)s.joint_angle_deg[0],(double)s.joint_angle_deg[1],(double)s.joint_angle_deg[2]);
}
static void ReportClosedLoop(void)
{
    ClosedLoopStatus_t s;
    MotionService_GetClosedLoopStatus(&s);
    SafetyServiceStatus_t safety;
    SafetyService_GetStatus(&safety);
    uint32_t now = PlatformTime_NowMs(), age[3];
    for (unsigned i = 0; i < 3; i++)
        age[i] = (s.decision_mask & (1U << i)) ? now - s.decision_ms[i] : UINT32_MAX;
    bool reference = s.target_initialized && safety.homed && !safety.fault_latched;
    /* CONTROL仅表示当前调度允许自主补偿；PS2、回零、锁定或故障时为0。
     * 诊断仅复制上次判定，M409仍负责原始反馈新鲜度；查询本身不推动闭环。 */
    bool control = reference && s_mode == APP_SERIAL_READY && safety.motion_allowed &&
                   App_Teleop_GetMode() == SYS_MODE_GCODE;
    Reply("M411 REF:%u CONTROL:%u ACTIVE:%u ENABLED_MASK:%u DECISION_MASK:%u AGE_MS:%lu,%lu,%lu TARGET_MOTOR:%.6g,%.6g,%.6g ERROR_MOTOR:%.6g,%.6g,%.6g ROUNDS:%lu,%lu,%lu LAST_STEPS:%ld,%ld,%ld\r\n",
          reference?1U:0U, control?1U:0U, s.active?1U:0U, (unsigned)s.enabled_mask,
          (unsigned)s.decision_mask, (unsigned long)age[0], (unsigned long)age[1], (unsigned long)age[2],
          (double)s.target_motor_deg[0], (double)s.target_motor_deg[1], (double)s.target_motor_deg[2],
          (double)s.error_motor_deg[0], (double)s.error_motor_deg[1], (double)s.error_motor_deg[2],
          (unsigned long)s.accepted_rounds[0], (unsigned long)s.accepted_rounds[1], (unsigned long)s.accepted_rounds[2],
          (long)s.last_steps[0], (long)s.last_steps[1], (long)s.last_steps[2]);
}

static void ReportTeleopDiagnostics(void)
{
    AppTeleopDiagnostics_t d;AppHardware_GetTeleopDiagnostics(&d);
    char last[19],first[19];
    for(unsigned i=0;i<9;i++){
        (void)snprintf(last+2*i,3,"%02X",d.last_frame[i]);
        (void)snprintf(first+2*i,3,"%02X",d.first_invalid_frame[i]);
    }
    /* 故障后仍可取首坏帧；只复制内存，不读取PS2总线、不清除现场。 */
    Reply("M412 FRAMES:%lu VALID:%lu INVALID:%lu STATUS:%u CONFIG:%u LAST_BAD_MS:%lu FIRST_RECORDED:%u FIRST_STATUS:%u FIRST_MS:%lu LAST:%s FIRST:%s\r\n",
        (unsigned long)d.frames,(unsigned long)d.valid_frames,(unsigned long)d.invalid_frames,
        d.last_status,d.config_phase,(unsigned long)d.last_invalid_ms,d.first_invalid_recorded,
        d.first_invalid_status,(unsigned long)d.first_invalid_ms,last,first);
}

#if defined(ROBOTARM_JOINT_RETURN_TEST) && ROBOTARM_JOINT_RETURN_TEST
static bool HandleJointTrial(const char *line)
{
    static const char *const phases[] = {"ARMED", "LOW", "RETURNING", "DONE"};
    if (strcmp(line, "M413") == 0) {
        Reply("M413 PHASE:%s GOAL_STEPS:0,%d,%d DURATION_MS:%u\r\n",
              phases[s_joint_trial], JOINT_TRIAL_M2, JOINT_TRIAL_M3,
              (unsigned)(JOINT_TRIAL_FRAMES * JOINT_TRIAL_FRAME_MS));
        return true;
    }
    const bool down = strcmp(line, "M413 DOWN") == 0;
    const bool up = strcmp(line, "M413 UP") == 0;
    if (!down && !up) return false;

    if (s_mode != APP_SERIAL_READY || App_Teleop_GetMode() != SYS_MODE_GCODE ||
        s_wait_move || s_wait_plan || !MotionService_IsActionAllowed() || !MotionCompleted() ||
        (down ? s_joint_trial != JOINT_TRIAL_ARMED : s_joint_trial != JOINT_TRIAL_LOW)) {
        Reply("error: joint test requires the expected settled pose and permission\r\n");
        return true;
    }
    for (unsigned axis = 0U; axis < MOTION_AXIS_COUNT; ++axis) {
        if (AppHardware_IsLimitTriggered(axis)) {
            Reply("error: joint test limit already triggered\r\n");
            return true;
        }
    }
    int32_t m1, m2, m3;
    MotionService_GetTheorySteps(&m1, &m2, &m3);
    if (m1 != 0 || m2 != (down ? 0 : JOINT_TRIAL_M2) || m3 != (down ? 0 : JOINT_TRIAL_M3)) {
        Reply("error: wrong joint test starting steps\r\n");
        return true;
    }
    if (down) {
        float x, y, z;
        App_GCodeExec_GetPlannedPosition(&x, &y, &z);
        if (fabsf(x - HOMEPOSE_X_MM) > 0.05f || fabsf(y - HOMEPOSE_Y_MM) > 0.05f ||
            fabsf(z - HOMEPOSE_Z_MM) > 0.05f) {
            Reply("error: joint test requires the home photo pose\r\n");
            return true;
        }
    }
    GripperStatus_t gripper;
    Svc_Gripper_GetStatus(&gripper);
    if (gripper.pwm_requested) {
        Reply("error: joint test requires an idle gripper\r\n");
        return true;
    }

    const int32_t goal2 = down ? JOINT_TRIAL_M2 : 0;
    const int32_t goal3 = down ? JOINT_TRIAL_M3 : 0;
    int32_t previous2 = m2;
    int32_t previous3 = m3;
    for (unsigned frame = 1U; frame <= JOINT_TRIAL_FRAMES; ++frame) {
        const float u = (float)frame / JOINT_TRIAL_FRAMES;
        const float progress = u*u*u * (10.0f + u * (6.0f*u - 15.0f));
        const int32_t next2 = m2 + (int32_t)lroundf((float)(goal2 - m2) * progress);
        const int32_t next3 = m3 + (int32_t)lroundf((float)(goal3 - m3) * progress);
        const MotionFrame_t segment = {
            .delta_m1 = 0, .delta_m2 = next2 - previous2, .delta_m3 = next3 - previous3,
            .total_ticks = JOINT_TRIAL_FRAME_MS * TICKS_PER_MS,
        };
        if (!MotionService_SubmitFrame(&segment)) {
            MotionService_AbortForFault(MOTION_FAULT_QUEUE_TIMEOUT);
            Reply("error: failed to submit joint test frames\r\n");
            return true;
        }
        previous2 = next2;
        previous3 = next3;
    }
    MotionService_AdjustTheorySteps(0, goal2 - m2, goal3 - m3);
    MotionService_SyncClosedLoopTarget();
    s_joint_trial = down ? JOINT_TRIAL_LOW : JOINT_TRIAL_RETURNING;
    Reply("ok\r\n");
    return true;
}
#endif

void AppSerial_Service(void)
{
#if defined(ROBOTARM_JOINT_RETURN_TEST) && ROBOTARM_JOINT_RETURN_TEST
    if (s_joint_trial == JOINT_TRIAL_RETURNING && MotionCompleted()) s_joint_trial = JOINT_TRIAL_DONE;
#endif
    PollPendingReply();
    char line[256];
    bool available = AppHardware_ReadLine(line,sizeof(line));
    bool bad_line = false;
    if (AppHardware_TakeLineTimeout()) {
        Reply("error: incomplete command; line ending required\r\n"); bad_line=true;
    }
    if (AppHardware_TakeRxOverflow()) {
        Reply("error: UART RX overflow; command discarded\r\n"); bad_line=true;
    }
    if (bad_line) {
        if (s_mode==APP_SERIAL_WAVEFORM) {
            WaveformService_Stop(WAVE_SERIAL); AppHardware_DiscardRx();
        }
        return;
    }
    if (!available) return;
    /* 诊断只接受精确查询，未知格式不会进入动作或恢复解析路径。 */
    if (s_mode==APP_SERIAL_DIAGNOSTIC && strcmp(line,"M115")!=0 &&
        strcmp(line,"M408")!=0 && strcmp(line,"M114")!=0 && strcmp(line,"M119")!=0 && strcmp(line,"M409")!=0 && strcmp(line,"M410")!=0 && strcmp(line,"M411")!=0 && strcmp(line,"M412")!=0) {
        Reply("error: diagnostic mode is read-only\r\n"); return;
    }
    /* 波形请求使用独立严格格式，不进入普通 G-code 执行器。 */
    if (s_mode==APP_SERIAL_WAVEFORM &&
        strcmp(line,"M115")!=0 && strcmp(line,"M408")!=0 &&
        strcmp(line,"M114")!=0 && strcmp(line,"M119")!=0 && strcmp(line,"M409")!=0 && strcmp(line,"M410")!=0 && strcmp(line,"M411")!=0 && strcmp(line,"M412")!=0) {
        AppWaveform_HandleCommand(line); return;
    }
#if ROBOTARM_SEQUENTIAL_HOMING
    if(strcmp(line,"M127")==0){
        SvcHomingSequenceStatus s;Svc_HomingSequence_GetStatus(&s);
        Reply("M127 CONFIRM_OTHER:%u SAMPLE_LOW:%u FAILURE_LOW:%u FAILURE_MS:%lu WATCH:%u EXPECT_LOW:%u PAUSE_MS:%lu SEEK_PERIOD_MS:%lu BACK_PERIOD_MS:%lu CONTACT_CANCELS:%lu CONTACT_HIGH:%u\r\n",
            s.flow.config.qualify_unexpected,s.flow.last_low,s.flow.failure_low,(unsigned long)s.flow.failure_ms,
            s.flow.hold_watch,s.flow.hold_expect_low,(unsigned long)s.flow.back_pause_ms,
            (unsigned long)s.flow.config.period_ms,(unsigned long)s.flow.config.backoff_period_ms,
            (unsigned long)s.flow.contact_cancel_count,s.flow.contact_high_seen);return;
    }
    if(strcmp(line,"M126")==0){
        SvcHomingSequenceStatus s;Svc_HomingSequence_GetStatus(&s);
        Reply("M126 PHASE:%u REASON:%u AXIS:%u CONTACTS:%u COMPLETE:%u OWNED:%u REARMED:%u SEEK:%lu,%lu,%lu BACK:%lu,%lu,%lu HOLD:%u HOLDS:%lu COMMITTED:%u BOOT_READY:%u PARALLEL:%u STEP_MASK:%u DIR_MASK:%u\r\n",
            s.flow.phase,s.flow.reason,s.flow.axis,s.flow.contacts,s.flow.completed,s.flow.owned,s.flow.rearmed,
            (unsigned long)s.flow.seek[0],(unsigned long)s.flow.seek[1],(unsigned long)s.flow.seek[2],
            (unsigned long)s.flow.back[0],(unsigned long)s.flow.back[1],(unsigned long)s.flow.back[2],
            s.flow.holding,(unsigned long)s.flow.hold_count,s.committed,s.boot_ready,
            s.flow.parallel,s.flow.step_mask,s.flow.dir_mask);return;
    }
#endif
#if defined(ROBOTARM_JOINT_RETURN_TEST) && ROBOTARM_JOINT_RETURN_TEST
    if (HandleJointTrial(line)) return;
    if (AppSerial_JointTrialActive() && strcmp(line, "M114") == 0) {
        Reply("error: Cartesian plan unavailable during joint test\r\n");
        return;
    }
#endif
    GCodeFrame_t frame;
    if (!App_GCodeParser_ParseLine(line,&frame)) { Reply("error: Parse failed!\r\n"); return; }

    /* 查询与停止即使在等待 M400、路径校验、回零或 PS2 模式下也可处理。 */
    if (frame.type==GCMD_M115) {
        if (s_mode==APP_SERIAL_DIAGNOSTIC) {
            Reply("M115 FIRMWARE:RobotArmDiagnostics PROTOCOL:DIAG3 UART:115200 MOTOR_ENABLE:LOCKED ENCODER_POLL:%u FEATURES:LIMIT_OBSERVATION,IRQ_FUSE,FEEDBACK_AGE,GRIPPER_STATUS,CLOSED_LOOP_STATUS\r\n",
                  AppDiagnostics_EncoderSamplingEnabled()?1U:0U);
            return;
        }
        if (s_mode==APP_SERIAL_WAVEFORM) {
            Reply("M115 FIRMWARE:RobotArmWaveform PROTOCOL:WAVE7 UART:115200 MOTOR_ENABLE:LOCKED FEATURES:TMC_QUERY,LIMIT_TRACE,IRQ_GUARD,VECTOR_TRACE,ENCODER_MASK,TMC_HEALTH,FEEDBACK_AGE,GRIPPER_STATUS,CLOSED_LOOP_STATUS\r\n");
            return;
        }
#if ROBOTARM_STARTUP_AUDIT_MODE
#if ROBOTARM_IRQ_EVIDENCE_MODE
        Reply("M115 FIRMWARE:RobotArmStartupAudit PROTOCOL:AUDIT1 AXES:3 UART:115200 MOTOR_ENABLE:LOCKED AUTO_MOTION:0 FEATURES:FEEDBACK_AGE,CLOSED_LOOP_STATUS,GRIPPER_STATUS,IRQ_EVIDENCE\r\n");
#else
        Reply("M115 FIRMWARE:RobotArmStartupAudit PROTOCOL:AUDIT1 AXES:3 UART:115200 MOTOR_ENABLE:LOCKED AUTO_MOTION:0 FEATURES:FEEDBACK_AGE,GRIPPER_STATUS,CLOSED_LOOP_STATUS\r\n");
#endif
#else
#if ROBOTARM_SEQUENTIAL_HOMING
#if ROBOTARM_LIMIT_STATE_MODE
        Reply("M115 FIRMWARE:RobotArm PROTOCOL:1 AXES:3 UART:115200 FEATURES:GCODE,STATUS,STOP,DIAGNOSTIC,LIMIT_STATE_1MS,FEEDBACK_AGE,GRIPPER_STATUS,SEQUENTIAL_HOME,M126,CLOSED_LOOP_STATUS,PS2_DIAGNOSTICS%s\r\n",
#if ROBOTARM_PARALLEL_HOMING
              ",PARALLEL_HOME"
#else
              ""
#endif
        );
#else
        Reply("M115 FIRMWARE:RobotArm PROTOCOL:1 AXES:3 UART:115200 FEATURES:GCODE,STATUS,STOP,DIAGNOSTIC,IRQ_GUARD,FEEDBACK_AGE,GRIPPER_STATUS,SEQUENTIAL_HOME,M126,CLOSED_LOOP_STATUS,PS2_DIAGNOSTICS%s\r\n",
#if ROBOTARM_PARALLEL_HOMING
              ",PARALLEL_HOME"
#else
              ""
#endif
        );
#endif
#else
        Reply("M115 FIRMWARE:RobotArm PROTOCOL:1 AXES:3 UART:115200 FEATURES:GCODE,STATUS,STOP,DIAGNOSTIC,IRQ_GUARD,FEEDBACK_AGE,GRIPPER_STATUS,CLOSED_LOOP_STATUS\r\n");
#endif
#endif
        return;
    }
    if (frame.type==GCMD_M408) { ReportStatus(); return; }
    if (frame.type==GCMD_M409) { ReportFeedback(); return; }
    if (frame.type==GCMD_M411) { ReportClosedLoop(); return; }
    if (frame.type==GCMD_M412) { ReportTeleopDiagnostics(); return; }
    if (frame.type==GCMD_M410) {
        /* 只读软件记录；不把PWM停止或目标值当作位置传感器结果。 */
        GripperStatus_t s;Svc_Gripper_GetStatus(&s);
        static const char *targets[]={"NONE","OPEN","CLOSE","CUSTOM"};
        const char *target=(unsigned)s.target<4U?targets[s.target]:"INVALID";
        Reply("M410 BOUND:%u TARGET:%s ANGLE_DEG:%.6g PWM_REQUESTED:%u HOLD_MS:%lu ELAPSED_MS:%lu ERROR:%u POSITION:UNMEASURED\r\n",
              s.bound?1U:0U,target,(double)s.target_angle_deg,s.pwm_requested?1U:0U,
              (unsigned long)s.hold_ms,(unsigned long)s.elapsed_ms,(unsigned)s.error);
        return;
    }
    if (frame.type==GCMD_M114) { ReportPosition(); return; }
    if (frame.type==GCMD_M119) {
        Reply("M119 M1:%s M2:%s M3:%s\r\n",
              AppHardware_IsLimitTriggered(0)?"TRIGGERED":"OPEN",
              AppHardware_IsLimitTriggered(1)?"TRIGGERED":"OPEN",
              AppHardware_IsLimitTriggered(2)?"TRIGGERED":"OPEN");
        return;
    }
    if (s_mode==APP_SERIAL_DIAGNOSTIC) {
        Reply("error: diagnostic mode is read-only\r\n"); return;
    }
    if (frame.type==GCMD_M112) {
        s_stop_request=true;
        /* 撤销所有未来动作，而非只清空 STEP 队列；避免停止后旧路径继续生成。 */
        Svc_Homing_Cancel();
        MotionService_AbortForFault(MOTION_FAULT_HOST_STOP);
        CommandService_AbortPending();
        SafetyService_ReportHostStop();
        s_wait_move=s_wait_plan=false;
        AppHardware_DiscardRx();
        Reply("M112OK\r\n"); return;
    }
    if (frame.type==GCMD_M999) {
        if (LimitIrqGuard_GetLatchMask()!=0U) {
            Reply("error: limit IRQ overload latched; restart required\r\n"); return;
        }
        if (!SafetyService_HasFault()) { Reply("error: M999 requires a safety fault\r\n"); return; }
        if (s_mode==APP_SERIAL_HOMING || s_mode==APP_SERIAL_CHECKING || s_mode==APP_SERIAL_OUTPUT_LOCKED) {
            Reply("error: startup or homing already in progress\r\n"); return;
        }
        AppSerial_SetMode(APP_SERIAL_HOMING);
        AppHardware_DiscardRx();
        s_recovery_request=true;
#if ROBOTARM_STARTUP_AUDIT_MODE
        Reply("M999: startup checks started; outputs locked\r\n"); return;
#else
        Reply("M999: homing started\r\n"); return;
#endif
    }
    if (s_mode!=APP_SERIAL_READY) { Reply("error: system not ready\r\n"); return; }
    if (App_Teleop_GetMode()!=SYS_MODE_GCODE) { Reply("error: PS2 mode; command rejected\r\n"); return; }
    if (s_wait_plan || s_wait_move) { Reply("error: previous command pending\r\n"); return; }
    if (frame.type==GCMD_M400) {
        if (SafetyService_HasFault()) Reply("error: M400 rejected by safety fault\r\n");
        else if (MotionCompleted()) Reply("ok\r\n");
        else s_wait_move=true;
        return;
    }
#if defined(ROBOTARM_JOINT_RETURN_TEST) && ROBOTARM_JOINT_RETURN_TEST
    if (AppSerial_JointTrialActive()) {
        Reply("error: joint test must return before normal commands\r\n");
        return;
    }
#endif
    ErrorCode_t result=App_GCodeExec_Run(&frame);
    if (result==ERR_PENDING) s_wait_plan=true;
    else if (result==ERR_QUEUED) Reply("queued\r\n");
    else if (result!=ERR_OK) Reply("error: command rejected (%d)\r\n",(int)result);
    else if (frame.type==GCMD_M3) Reply("M3OK\r\n");
    else if (frame.type==GCMD_M5) Reply("M5OK\r\n");
    else {
        if (frame.type==GCMD_G0 || frame.type==GCMD_G1) MotionService_SyncClosedLoopTarget();
        Reply("ok\r\n");
    }
}
