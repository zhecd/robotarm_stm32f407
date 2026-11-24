/** 真实PS2命令→规划→G-code缺省坐标继承；只替换设备和时间。 */
#include "command_service.h"
#include "motion_service.h"
#include "planning_service.h"
#include "safety_service.h"
#include "state_service.h"
#include "fake_robot.h"
#include "robot_config.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);exit(1);} } while(0)
static void Drain(void)
{
    unsigned n;
    for(n=0;n<200000 && (CommandService_IsMotionPending() || !MotionService_IsIdle());n++) {
        CommandService_Service();ErrorCode_t result;
        if(CommandService_TakeMoveResult(&result))CHECK(result==ERR_OK);
        if(CommandService_TakeQueuedMoveResult(&result))CHECK(result==ERR_OK);
        for(unsigned k=0;k<100;k++)MotionService_OnStepTickFromISR();
    }
    CHECK(n<200000);
}
static CommandServiceStatus_t Status(void)
{CommandServiceStatus_t s;CommandService_GetStatus(&s);return s;}
int main(int argc,char **argv)
{
    CHECK(argc==2);const char *t=argv[1];
    FakeRobot_Reset();StateService_Init();SafetyService_Init();MotionService_Init();MotionService_InitClosedLoop();
    CHECK(CommandService_Init(HOMEPOSE_X_MM,HOMEPOSE_Y_MM,HOMEPOSE_Z_MM)==ERR_OK);
    SafetyService_MarkHomed();SafetyService_ClearAfterSuccessfulHoming();
    if(!strcmp(t,"tiny")) {
        for(unsigned n=0;n<1000;n++){CHECK(CommandService_RunTeleopStep(.001f,0,0)==ERR_OK);Drain();}
        CHECK(fabsf(Status().planned_x_mm-1)<.001f);CHECK(fake_steps[0]>0);
    } else {
        CHECK(CommandService_RunTeleopStep(.9f,.5f,-.5f)==ERR_OK);Drain();
        CommandServiceStatus_t moved=Status();
        if(!strcmp(t,"rejected")) {
            CHECK(CommandService_RunTeleopStep(100000,0,0)==ERR_OUT_OF_RANGE);
            CHECK(Status().generation==moved.generation);
        }
        GCodeFrame_t f={.type=GCMD_G1,.has_f=true,.f=120};
        if(!strcmp(t,"x") || !strcmp(t,"rejected") || !strcmp(t,"queued")) {f.has_x=true;f.x=2;}
        else if(!strcmp(t,"y")){f.has_y=true;f.y=187;}
        else if(!strcmp(t,"z")){f.has_z=true;f.z=238;}
        else if(!strcmp(t,"feed_only")) { f.f=300; }
        else CHECK(false);
        ErrorCode_t rc=CommandService_RunGCode(&f);CHECK(rc==ERR_PENDING || rc==ERR_OK);
        if(!strcmp(t,"queued")) {
            GCodeFrame_t second={.type=GCMD_G1,.has_z=true,.z=238};
            CHECK(CommandService_RunGCode(&second)==ERR_QUEUED);
        }
        Drain();CommandServiceStatus_t end=Status();
        CHECK(fabsf(end.planned_x_mm-(f.has_x?f.x:moved.planned_x_mm))<.0001f);
        CHECK(fabsf(end.planned_y_mm-(f.has_y?f.y:moved.planned_y_mm))<.0001f);
        CHECK(fabsf(end.planned_z_mm-(!strcmp(t,"queued")?238:f.has_z?f.z:moved.planned_z_mm))<.0001f);
        CHECK(!MotionService_HasFault());
    }
    printf("teleop_coordinate_handoff %s passed\n",t);return 0;
}
