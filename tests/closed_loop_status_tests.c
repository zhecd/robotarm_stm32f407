/** 闭环诊断组合测试：真实控制/STEP/协议，替换设备和时间；不连接机械臂。 */
#include "motion_service.h"
#include "safety_service.h"
#include "state_service.h"
#include "app/app_serial_protocol.h"
#include "app/app_teleop.h"
#include "app/app_gcode_parser.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include "robot_config.h"
#include "robot_math.h"
#include <assert.h>
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned encoder_reads;
static bool boundary_status;
void __real_MotionService_GetClosedLoopStatus(ClosedLoopStatus_t *out);
void __wrap_MotionService_GetClosedLoopStatus(ClosedLoopStatus_t *out)
{
    /* 仅边界协议场景替换服务快照，检验真实格式化器在32位极值下完整输出。
     * 其余场景仍调用真实控制器，不能用这份快照代替补偿执行验证。 */
    if (!boundary_status) { __real_MotionService_GetClosedLoopStatus(out); return; }
    assert(out);
    *out = (ClosedLoopStatus_t){.target_initialized=true, .enabled_mask=7, .decision_mask=7};
    for (unsigned i = 0; i < 3; i++) {
        out->target_motor_deg[i] = -FLT_MAX;
        out->error_motor_deg[i] = -FLT_MAX;
        out->decision_ms[i] = 1U;
        out->accepted_rounds[i] = UINT32_MAX;
        out->last_steps[i] = INT32_MIN;
    }
}
static int32_t emitted[3];
ErrorCode_t __real_Dev_Joint_ReadMotorAngle(DevJointId_t axis, float *out);
ErrorCode_t __wrap_Dev_Joint_ReadMotorAngle(DevJointId_t axis, float *out)
{
    encoder_reads++;
    return __real_Dev_Joint_ReadMotorAngle(axis, out);
}
bool __real_Dev_Joint_Step(DevJointId_t axis);
bool __wrap_Dev_Joint_Step(DevJointId_t axis)
{
    bool ok = __real_Dev_Joint_Step(axis);
    if (ok) emitted[axis] += fake_clockwise[axis] ? 1 : -1;
    return ok;
}
static ClosedLoopStatus_t Status(void)
{
    ClosedLoopStatus_t s;
    MotionService_GetClosedLoopStatus(&s);
    return s;
}
static void Reset(void)
{
    FakeRobot_Reset(); FakeHardware_Reset(); StateService_Init(); SafetyService_Init();
    MotionService_Init(); MotionService_InitClosedLoop();
    SafetyService_MarkHomed(); SafetyService_ClearAfterSuccessfulHoming();
    App_Teleop_SetMode(SYS_MODE_GCODE); App_Teleop_Init();
    AppSerial_Init(APP_SERIAL_READY);
    encoder_reads = 0;
    memset(emitted, 0, sizeof(emitted));
}
static void Decide(float angle)
{
    MotionService_SyncClosedLoopTarget();
    fake_motor_deg[0] = angle;
    FakeRobot_AdvanceUs((CL_SETTLE_MS + CL_COOLDOWN_LO_MS) * 1000U);
    MotionService_UpdateClosedLoop();
}
static void Query(void)
{
    unsigned before = encoder_reads;
    uint16_t queued = MotionService_GetQueueCount();
    uint32_t steps[3] = {fake_steps[0], fake_steps[1], fake_steps[2]};
    unsigned enables = fake_enable_true_count;
    FakeHardware_ClearTx(); FakeHardware_SendLine("M411"); AppSerial_Service();
    assert(strstr(fake_tx, "M411 ") && !strstr(fake_tx, "error:"));
    assert(strlen(fake_tx) < 384);
    assert(encoder_reads == before && queued == MotionService_GetQueueCount());
    for (unsigned i = 0; i < 3; i++) assert(steps[i] == fake_steps[i]);
    assert(fake_enable_true_count == enables);
    assert(fake_gripper_open_count == 0 && fake_gripper_close_count == 0);
}
int main(int argc, char **argv)
{
    assert(argc == 2); Reset(); const char *test = argv[1];
    if (!strcmp(test, "init")) {
        ClosedLoopStatus_t s = Status();
        assert(s.target_initialized && !s.active && s.decision_mask == 0 && s.enabled_mask == 7);
        for (unsigned i = 0; i < 3; i++) assert(s.accepted_rounds[i] == 0 && s.last_steps[i] == 0);
        MotionService_GetClosedLoopStatus(NULL);
        SafetyService_Init(); /* Init默认零目标不能替代真正回零许可。 */
        Query(); assert(strstr(fake_tx, "REF:0 CONTROL:0 ACTIVE:0"));
    } else if (!strcmp(test, "deadband")) {
        Decide(CL_DEADBAND_DEG); ClosedLoopStatus_t s = Status();
        assert(s.target_initialized && s.decision_mask == 7 && !s.active);
        assert(fabsf(s.error_motor_deg[0] + CL_DEADBAND_DEG) < .001f);
        assert(s.accepted_rounds[0] == 0 && s.last_steps[0] == 0);
        Query(); assert(strstr(fake_tx, "REF:1 CONTROL:1 ACTIVE:0"));
    } else if (!strcmp(test, "small") || !strcmp(test, "recovery")) {
        Decide(!strcmp(test, "small") ? 4.0f : -10.0f);
        ClosedLoopStatus_t accepted = Status();
        assert(accepted.active && accepted.accepted_rounds[0] == 1);
        assert(accepted.last_steps[0] != 0 && fake_steps[0] == 0);
        assert(accepted.accepted_rounds[1] == 0 && accepted.last_steps[1] == 0);
        /* 受理后尚未发出STEP：状态不能冒充实际步数。随后核对真实引擎输出。 */
        Query(); assert(strstr(fake_tx, "ACTIVE:1"));
        for (unsigned i = 0; i < 4000 && MotionService_IsClosedLoopRecoveryActive(); i++) {
            for (unsigned k = 0; k < 1000; k++) MotionService_OnStepTickFromISR();
            FakeRobot_AdvanceUs(CL_UPDATE_MS * 1000U); MotionService_UpdateClosedLoop();
        }
        ClosedLoopStatus_t done = Status();
        assert(!done.active && emitted[0] == accepted.last_steps[0]);
        assert(done.accepted_rounds[0] == 1 && emitted[1] == 0 && emitted[2] == 0);
        int32_t a, b, c; MotionService_GetTheorySteps(&a, &b, &c);
        assert(a == 0 && b == 0 && c == 0);
    } else if (!strcmp(test, "stop")) {
        Decide(10); ClosedLoopStatus_t accepted = Status();
        MotionService_AbortForFault(MOTION_FAULT_HOST_STOP);
        ClosedLoopStatus_t stopped = Status(); /* 无需等待下一次Update。 */
        assert(!stopped.active && !stopped.target_initialized && stopped.decision_mask == 0);
        assert(stopped.accepted_rounds[0] == accepted.accepted_rounds[0]);
        assert(stopped.last_steps[0] == accepted.last_steps[0] && fake_steps[0] == 0);
        Query(); assert(strstr(fake_tx, "REF:0 CONTROL:0 ACTIVE:0"));
    } else if (!strcmp(test, "sync")) {
        Decide(4); MotionService_AbortForFault(MOTION_FAULT_HOST_STOP);
        MotionService_ClearFault(); MotionService_AdjustTheorySteps(16, -32, 48);
        MotionService_SyncClosedLoopTarget(); ClosedLoopStatus_t s = Status();
        assert(s.target_initialized && !s.active && s.decision_mask == 0);
        assert(fabsf(s.target_motor_deg[0] - StepsToDeg(16)) < .001f);
        assert(fabsf(s.target_motor_deg[1] - StepsToDeg(-32)) < .001f);
        assert(s.accepted_rounds[0] == 1);
        MotionService_InitClosedLoop(); assert(Status().accepted_rounds[0] == 0);
    } else if (!strcmp(test, "failed_read")) {
        Decide(0); assert(Status().decision_mask == 7);
        fake_encoder_failure[1] = true;
        FakeRobot_AdvanceUs(CL_UPDATE_MS * 1000U); MotionService_UpdateClosedLoop();
        assert(Status().decision_mask == 5 && !MotionService_HasFault());
        Query(); assert(strstr(fake_tx, "DECISION_MASK:5"));
    } else if (!strcmp(test, "age_wrap")) {
        FakeRobot_SetTimeUs(((uint64_t)UINT32_MAX - 1000U) * 1000U);
        Decide(0); ClosedLoopStatus_t s = Status();
        FakeRobot_SetTimeUs(((uint64_t)s.decision_ms[0] + 1200U) * 1000U);
        Query(); assert(strstr(fake_tx, "AGE_MS:1200,1200,1200"));
    } else if (!strcmp(test, "ps2")) {
        Decide(0); App_Teleop_SetMode(SYS_MODE_PS2);
        Query(); assert(strstr(fake_tx, "REF:1 CONTROL:0"));
    } else if (!strcmp(test, "disabled")) {
        Decide(0); MotionService_SetClosedLoopAxisEnabled(1, false);
        Query(); assert(strstr(fake_tx, "ENABLED_MASK:5"));
    } else if (!strcmp(test, "bounds")) {
        boundary_status = true; Query();
        assert(strstr(fake_tx, "ROUNDS:4294967295,4294967295,4294967295"));
        assert(strstr(fake_tx, "LAST_STEPS:-2147483648,-2147483648,-2147483648\r\n"));
        assert(strstr(fake_tx, "AGE_MS:4294967295,4294967295,4294967295"));
    } else if (!strcmp(test, "syntax")) {
        const char *bad[] = {"M411 X1", "M411 M112", "M411.1", "M411 S1"};
        GCodeFrame_t f;
        assert(App_GCodeParser_ParseLine(" m411 \r\n", &f) && f.type == GCMD_M411);
        for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); i++) {
            FakeHardware_ClearTx(); FakeHardware_SendLine(bad[i]); AppSerial_Service();
            assert(strstr(fake_tx, "error:") && !AppSerial_TakeStopRequest());
        }
    } else if (!strncmp(test, "mode", 4)) {
        unsigned mode = (unsigned)strtoul(test + 4, NULL, 10);
        assert(mode <= APP_SERIAL_OUTPUT_LOCKED);
        SafetyService_Init(); /* 各启动阶段尚未建立Safety参考。 */
        AppSerial_Init((AppSerialMode_t)mode); Query();
        assert(strstr(fake_tx, "REF:0 CONTROL:0 ACTIVE:0"));
        assert(!AppSerial_TakeRecoveryRequest() && !AppSerial_TakeStopRequest());
    } else { assert(false); }
    printf("closed_loop_status (%s): passed\n", test); return 0;
}
