/** 在真实闭环入口注入设备读取耗时，验证旧角度不会用于补偿。 */
#include "motion_service.h"
#include "state_service.h"
#include "safety_service.h"
#include "fake_robot.h"
#include "robot_config.h"
#include "platform_time.h"
#include "ctrl_motion_engine.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static uint32_t delay_ms;
ErrorCode_t __real_Dev_Joint_ReadMotorAngle(DevJointId_t axis,float *angle);
ErrorCode_t __wrap_Dev_Joint_ReadMotorAngle(DevJointId_t axis,float *angle)
{
    FakeRobot_AdvanceUs(delay_ms*1000U);
    return __real_Dev_Joint_ReadMotorAngle(axis,angle);
}
static void Reset(void)
{
    delay_ms=0;FakeRobot_Reset();StateService_Init();SafetyService_Init();MotionService_Init();
    MotionService_InitClosedLoop();MotionService_SyncClosedLoopTarget();
}
/* 等待机械稳定只能延迟补偿，不能冻结对上位机发布的反馈。 */
static void CheckSettleSampling(void)
{
    for (unsigned elapsed=CL_UPDATE_MS; elapsed<CL_SETTLE_MS; elapsed+=CL_UPDATE_MS) {
        FakeRobot_AdvanceUs(CL_UPDATE_MS*1000U);
        MotionService_UpdateClosedLoop();
        StateServiceStatus_t s;StateService_GetStatus(&s);
        assert(StateService_FreshMask(&s,PlatformTime_NowMs(),FEEDBACK_QUERY_MAX_AGE_MS)==7U);
        assert(!MotionService_HasFault());
        assert(MotionService_GetQueueCount()==0 && !MotionService_IsClosedLoopRecoveryActive());
        assert(fake_steps[0]==0 && fake_steps[1]==0 && fake_steps[2]==0);
    }
}
static void TestSettleFeedback(void)
{
    Reset();fake_motor_deg[0]=10.0f;
    CheckSettleSampling(); /* SyncTarget之后，即使有误差也只采样。 */
    FakeRobot_AdvanceUs(CL_UPDATE_MS*1000U);MotionService_UpdateClosedLoop();
    assert(MotionService_IsClosedLoopRecoveryActive()); /* 等待到期后仍可补偿。 */

    Reset();
    MotionFrame_t wait={0,0,0,1};
    assert(Ctrl_MotionEngine_PushFrame(&wait));
    MotionService_UpdateClosedLoop(); /* 经过真实队列的busy分支。 */
    MotionService_OnStepTickFromISR();
    assert(MotionService_IsIdle());
    fake_motor_deg[0]=10.0f;
    MotionService_UpdateClosedLoop(); /* busy转idle，开始新的稳定等待。 */
    CheckSettleSampling();

    Reset();
    FakeRobot_SetTimeUs(((uint64_t)UINT32_MAX-100U)*1000U);
    MotionService_SyncClosedLoopTarget();fake_motor_deg[0]=10.0f;
    CheckSettleSampling(); /* 毫秒计数回绕不能中断采样或提前补偿。 */

    Reset();fake_encoder_failure[1]=true;
    for(unsigned i=0;i<CL_ENCODER_FAIL_LIMIT;i++) {
        FakeRobot_AdvanceUs(CL_UPDATE_MS*1000U);MotionService_UpdateClosedLoop();
    }
    assert(MotionService_GetFaultReason()==MOTION_FAULT_ENCODER);
    assert(MotionService_GetQueueCount()==0 && fake_steps[0]==0);

    Reset();fake_motor_deg[0]=10000.0f;
    FakeRobot_AdvanceUs(CL_UPDATE_MS*1000U);MotionService_UpdateClosedLoop();
    assert(MotionService_GetFaultReason()==MOTION_FAULT_SOFT_LIMIT);
    assert(MotionService_GetQueueCount()==0 && fake_steps[0]==0);
}
int main(void)
{
    TestSettleFeedback();
    Reset();delay_ms=CL_FEEDBACK_MAX_READ_MS-1U;float angle;
    assert(MotionService_GetClosedLoopAxisAngle(0,&angle));
    StateServiceStatus_t s;StateService_GetStatus(&s);
    assert(s.sample_finished_ms[0]-s.sample_started_ms[0]==delay_ms);
    assert(StateService_FreshMask(&s,PlatformTime_NowMs(),100)&1U);
    Reset();delay_ms=CL_FEEDBACK_MAX_READ_MS;
    for(unsigned i=0;i<CL_ENCODER_FAIL_LIMIT;i++)assert(!MotionService_GetClosedLoopAxisAngle(0,&angle));
    assert(MotionService_GetFaultReason()==MOTION_FAULT_ENCODER);
    StateService_GetStatus(&s);assert(!s.encoder_valid[0] && s.encoder_fail_count[0]==CL_ENCODER_FAIL_LIMIT);
    Reset();fake_motor_deg[1]=NAN;
    for(unsigned i=0;i<CL_ENCODER_FAIL_LIMIT;i++)assert(!MotionService_GetClosedLoopAxisAngle(1,&angle));
    assert(MotionService_GetFaultReason()==MOTION_FAULT_ENCODER && MotionService_GetQueueCount()==0);
    assert(fake_steps[0]==0 && fake_steps[1]==0 && fake_steps[2]==0);
    puts("closed_loop_feedback_timing_tests: passed");return 0;
}
