#include "command_service.h"
#include "motion_service.h"
#include "planning_service.h"
#include "safety_service.h"
#include "state_service.h"
#include "fake_robot.h"
#include "robot_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void Reset(void)
{
    FakeRobot_Reset(); MotionService_Init(); StateService_Init(); SafetyService_Init();
    MotionService_InitClosedLoop();
    assert(CommandService_Init(HOMEPOSE_X_MM,HOMEPOSE_Y_MM,HOMEPOSE_Z_MM)==ERR_OK);
    SafetyService_MarkHomed(); SafetyService_ClearAfterSuccessfulHoming();
}
static GCodeFrame_t Move(float x)
{
    return (GCodeFrame_t){.type=GCMD_G1,.x=x,.has_x=true};
}
static void Drain(void)
{
    unsigned i;
    for(i=0;i<200000U && (CommandService_IsMotionPending()||!MotionService_IsIdle());i++) {
        CommandService_Service();
        ErrorCode_t result;
        if (CommandService_TakeMoveResult(&result)) assert(result==ERR_OK);
        if (CommandService_TakeQueuedMoveResult(&result)) assert(result==ERR_OK);
        for(unsigned tick=0;tick<100;tick++) MotionService_OnStepTickFromISR();
    }
    assert(i<200000U);
}
int main(void)
{
    Reset(); GCodeFrame_t frame=Move(1);
    frame.has_f=true; frame.f=1000;
    assert(CommandService_RunGCode(&frame)==ERR_PENDING);
    for(unsigned i=0;i<COMMAND_WAYPOINT_QUEUE_SIZE;i++) {
        frame=Move(2.0f+i); assert(CommandService_RunGCode(&frame)==ERR_QUEUED);
    }
    assert(CommandService_GetQueuedMoveCount()==COMMAND_WAYPOINT_QUEUE_SIZE);
    frame=Move(50); assert(CommandService_RunGCode(&frame)==ERR_BUFFER_FULL);
    Drain();
    CommandServiceStatus_t status; CommandService_GetStatus(&status);
    assert(status.planned_x_mm==1.0f+COMMAND_WAYPOINT_QUEUE_SIZE && status.feedrate_mm_min==1000);
    frame=(GCodeFrame_t){.type=GCMD_G1,.y=186,.has_y=true};
    assert(CommandService_RunGCode(&frame)==ERR_PENDING); Drain();
    CommandService_GetStatus(&status);
    assert(status.planned_y_mm==186 && status.planned_x_mm==1.0f+COMMAND_WAYPOINT_QUEUE_SIZE);
    frame=Move(NAN); assert(CommandService_RunGCode(&frame)==ERR_OUT_OF_RANGE);
    frame=Move(1e30f); assert(CommandService_RunGCode(&frame)==ERR_OUT_OF_RANGE);
    frame=Move(1); frame.has_f=true; frame.f=0;
    assert(CommandService_RunGCode(&frame)==ERR_OUT_OF_RANGE);
    Reset(); frame=Move(1); assert(CommandService_RunGCode(&frame)==ERR_PENDING);
    frame=Move(2); assert(CommandService_RunGCode(&frame)==ERR_QUEUED);
    MotionService_AbortForFault(MOTION_FAULT_HOST_STOP); SafetyService_ReportHostStop();
    CommandService_Service();
    assert(!CommandService_IsMotionPending() && CommandService_GetQueuedMoveCount()==0);
    assert(!PlanningService_IsBusy());
    assert(CommandService_RunGCode(&frame)==ERR_BUSY);
    SafetyService_ClearAfterSuccessfulHoming(); assert(!SafetyService_IsMotionAllowed());
    puts("command_service_tests: passed"); return 0;
}
