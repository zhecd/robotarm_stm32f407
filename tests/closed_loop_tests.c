#include "motion_service.h"
#include "state_service.h"
#include "safety_service.h"
#include "fake_robot.h"
#include "robot_config.h"
#include <assert.h>
#include <stdio.h>
static void Reset(void)
{
    FakeRobot_Reset(); StateService_Init(); SafetyService_Init(); MotionService_Init();
    MotionService_InitClosedLoop(); MotionService_SyncClosedLoopTarget();
}
int main(void)
{
    Reset(); fake_encoder_failure[1]=true;
    float angle;
    for(unsigned i=0;i<CL_ENCODER_FAIL_LIMIT;i++) assert(!MotionService_GetClosedLoopAxisAngle(1,&angle));
    assert(MotionService_GetFaultReason()==MOTION_FAULT_ENCODER);
    StateServiceStatus_t state; StateService_GetStatus(&state);
    assert(!state.encoder_valid[1] && state.encoder_fail_count[1]==CL_ENCODER_FAIL_LIMIT);
    Reset(); fake_motor_deg[0]=10000;
    assert(!MotionService_GetClosedLoopAxisAngle(0,&angle));
    assert(MotionService_GetFaultReason()==MOTION_FAULT_SOFT_LIMIT);
    Reset(); fake_motor_deg[0]=10; FakeRobot_AdvanceUs(300000);
    MotionService_UpdateClosedLoop();
    assert(MotionService_IsClosedLoopRecoveryActive() && MotionService_GetQueueCount()>0);
    unsigned i;
    for(i=0;i<2000 && MotionService_IsClosedLoopRecoveryActive();i++) {
        for(unsigned tick=0;tick<1000;tick++) MotionService_OnStepTickFromISR();
        FakeRobot_AdvanceUs(20000); MotionService_UpdateClosedLoop();
    }
    assert(i<2000 && fake_steps[0]>0 && fake_steps[1]==0 && fake_steps[2]==0);
    int32_t a,b,c; MotionService_GetTheorySteps(&a,&b,&c);
    assert(a==0 && b==0 && c==0); /* 补偿返回原目标，不能改变规划位置。 */
    Reset(); MotionService_AbortForFault(MOTION_FAULT_ENCODER);
    MotionFaultReason_t reason;
    assert(MotionService_TakeFaultEvent(&reason) && reason==MOTION_FAULT_ENCODER);
    MotionService_ServiceSafety(); assert(!MotionService_TakeFaultEvent(&reason));
    MotionService_ClearFault(); MotionService_AbortForFault(MOTION_FAULT_HOST_STOP);
    assert(MotionService_TakeFaultEvent(&reason) && reason==MOTION_FAULT_HOST_STOP);
    puts("closed_loop_tests: passed"); return 0;
}
