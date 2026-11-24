#include <assert.h>
#include <stdio.h>

#include "planning_service.h"
#include "motion_service.h"
#include "robot_config.h"
#include "algorithm/kinematics.h"
#include <math.h>

#define TEST_FRAME_CAPACITY 4096U

static MotionFrame_t s_frames[TEST_FRAME_CAPACITY];
static unsigned int s_frame_count;
static bool s_faulted;
static bool s_reject_frames;
static uint32_t s_now_ms;
uint32_t PlatformTime_NowMs(void) { return s_now_ms; }
static int32_t s_theory_m1;
static int32_t s_theory_m2;
static int32_t s_theory_m3;

bool MotionService_SubmitFrame(const MotionFrame_t *frame)
{
    if (s_faulted || s_reject_frames || frame == NULL || s_frame_count >= TEST_FRAME_CAPACITY) return false;
    s_frames[s_frame_count++] = *frame;
    return true;
}

bool MotionService_HasFault(void)
{
    return s_faulted;
}

uint16_t MotionService_GetQueueCount(void)
{
    return 0U;
}

void MotionService_AdjustTheorySteps(int32_t dm1, int32_t dm2, int32_t dm3)
{
    s_theory_m1 += dm1;
    s_theory_m2 += dm2;
    s_theory_m3 += dm3;
}

void MotionService_AbortForFault(MotionFaultReason_t reason)
{
    (void)reason;
    s_faulted = true;
}

static void ResetMotionStub(void)
{
    s_frame_count = 0U;
    s_faulted = false;
    s_reject_frames = false;
    s_now_ms = 0;
    s_theory_m1 = 0;
    s_theory_m2 = 0;
    s_theory_m3 = 0;
}

static void TestPlanningFacade(void)
{
    ErrorCode_t result;
    ResetMotionStub();
    assert(PlanningService_Init(HOMEPOSE_X_MM, HOMEPOSE_Y_MM, HOMEPOSE_Z_MM) == ERR_OK);
    assert(PlanningService_StartLine(100.0f, 180.0f, 160.0f, 2000U) == ERR_PENDING);
    assert(PlanningService_IsBusy());
    assert(PlanningService_StartLine(110.0f, 180.0f, 160.0f, 2000U) == ERR_BUSY);

    do {
        PlanningService_Service();
    } while (!PlanningService_TakeStartResult(&result));
    assert(result == ERR_OK);

    while (PlanningService_IsBusy()) PlanningService_Service();
    assert(s_frame_count > 0U);
    assert(s_theory_m1 != 0 || s_theory_m2 != 0 || s_theory_m3 != 0);
}

static void TestUnreachableTarget(void)
{
    ResetMotionStub();
    assert(PlanningService_Init(HOMEPOSE_X_MM, HOMEPOSE_Y_MM, HOMEPOSE_Z_MM) == ERR_OK);
    assert(PlanningService_StartLine(0.0f, 185.0f, 1000.0f, 1000U) == ERR_OUT_OF_RANGE);
    assert(!PlanningService_IsBusy());
}

static void TestSlowLongMoveKeepsItsRequestedDuration(void)
{
    ErrorCode_t result;
    const uint32_t duration_ms = 19444U;
    uint64_t submitted_ticks = 0U;
    unsigned int zero_step_frames = 0U;

    ResetMotionStub();
    assert(PlanningService_Init(HOMEPOSE_X_MM, HOMEPOSE_Y_MM, HOMEPOSE_Z_MM) == ERR_OK);
    assert(PlanningService_StartLine(28.0f, 233.0f, 160.0f, duration_ms) == ERR_PENDING);

    do {
        PlanningService_Service();
    } while (!PlanningService_TakeStartResult(&result));
    assert(result == ERR_OK);

    while (PlanningService_IsBusy()) PlanningService_Service();
    assert(s_frame_count == 973U);
    for (unsigned int i = 0U; i < s_frame_count; ++i) {
        submitted_ticks += s_frames[i].total_ticks;
        if (s_frames[i].delta_m1 == 0 && s_frames[i].delta_m2 == 0 &&
            s_frames[i].delta_m3 == 0)
            zero_step_frames++;
    }
    assert(submitted_ticks >= (uint64_t)duration_ms * TICKS_PER_MS);
    assert(zero_step_frames > 0U);
}

static void TestTinyMoveCannotBypassJointLimit(void)
{
    ResetMotionStub();
    const float radius=185.0f;
    const float start_angle=96.99f*0.017453292519943295f;
    const float target_angle=97.01f*0.017453292519943295f;
    assert(PlanningService_Init(radius*sinf(start_angle),radius*cosf(start_angle),240)==ERR_OK);
    assert(PlanningService_StartLine(radius*sinf(target_angle),radius*cosf(target_angle),240,100U)==ERR_OUT_OF_RANGE);
    assert(!PlanningService_IsBusy() && s_frame_count==0U);
}

static void TestTinyMovesPreserveMotorReference(void)
{
    ResetMotionStub();
    RobotAngles_t angles; RobotMotorUnits_t start,finish;
    assert(Kinematics_Solve(0,185,240,&angles)==ERR_OK);
    Kinematics_ToMotorUnits(&angles,&start);
    assert(PlanningService_Init(0,185,240)==ERR_OK);
    for (unsigned move=1;move<=100U;move++) {
        ErrorCode_t result=PlanningService_StartLine((float)move*0.05f,185,240,3U);
        assert(result==ERR_OK || result==ERR_PENDING);
        unsigned budget=100U;
        bool accepted=result==ERR_OK;
        while(PlanningService_IsBusy() && budget--) {
            PlanningService_Service();
            if(PlanningService_TakeStartResult(&result)) { assert(result==ERR_OK);accepted=true; }
        }
        assert(budget>0U && accepted);
    }
    assert(Kinematics_Solve(5,185,240,&angles)==ERR_OK);
    Kinematics_ToMotorUnits(&angles,&finish);
    assert(s_theory_m1==finish.rot_units-start.rot_units);
    assert(s_theory_m2==finish.low_units-start.low_units);
    assert(s_theory_m3==finish.high_units-start.high_units);
    int32_t totals[3]={0};
    for(unsigned i=0;i<s_frame_count;i++) {
        totals[0]+=s_frames[i].delta_m1;totals[1]+=s_frames[i].delta_m2;totals[2]+=s_frames[i].delta_m3;
    }
    assert(totals[0]==s_theory_m1 && totals[1]==s_theory_m2 && totals[2]==s_theory_m3);
    assert(s_frame_count>0U);
    unsigned previous=s_frame_count;
    assert(PlanningService_StartLine(5,185,240,1U)==ERR_OK);
    assert(s_frame_count==previous && !PlanningService_IsBusy());
}

static void TestDurationOverflowIsRejectedBeforeStateChange(void)
{
    for(unsigned i=0;i<2U;i++) {
        ResetMotionStub();
        assert(PlanningService_Init(0,185,240)==ERR_OK);
        uint32_t duration=i==0U ? UINT32_MAX : UINT32_MAX/TICKS_PER_MS;
        assert(PlanningService_StartLine(10,185,240,duration)==ERR_OUT_OF_RANGE);
        assert(!PlanningService_IsBusy() && s_frame_count==0U);
    }
}

int main(void)
{
    TestTinyMoveCannotBypassJointLimit();
    TestTinyMovesPreserveMotorReference();
    TestDurationOverflowIsRejectedBeforeStateChange();
    TestPlanningFacade();
    TestUnreachableTarget();
    TestSlowLongMoveKeepsItsRequestedDuration();
    /* 队列不消费时先返回主循环，超过真实毫秒时限后才锁存故障。 */
    ResetMotionStub();
    assert(PlanningService_Init(HOMEPOSE_X_MM,HOMEPOSE_Y_MM,HOMEPOSE_Z_MM)==ERR_OK);
    assert(PlanningService_StartLine(10,185,240,1000)==ERR_PENDING);
    ErrorCode_t result=ERR_PENDING;
    for(unsigned i=0;i<1000 && !PlanningService_TakeStartResult(&result);i++) PlanningService_Service();
    assert(result==ERR_OK);
    s_reject_frames=true; PlanningService_Service(); assert(!s_faulted);
    s_now_ms=PLANNER_QUEUE_TIMEOUT_MS+1; PlanningService_Service();
    assert(s_faulted && !PlanningService_IsBusy());
    puts("planning_service_tests: passed");
    return 0;
}
