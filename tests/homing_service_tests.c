#include "service/svc_homing.h"
#include "fake_robot.h"
#include "robot_config.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
static void Tick(void) { FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US); Svc_Homing_Step(); }
static void Begin(void) { Svc_Homing_Cancel(); FakeRobot_Reset(); assert(Svc_Homing_Start()); }
static void FinishBackoff(void)
{
    for (unsigned i=0;i<HOMING_MAX_STEPS*4U && !Svc_Homing_IsFinished();++i) Tick();
    assert(Svc_Homing_IsFinished());
}
int main(void)
{
    Begin();
    assert(fake_clockwise[0]!=(bool)HOMING_SEEK_M1_DIR_HIGH);
    assert(fake_clockwise[1]!=(bool)HOMING_SEEK_M2_DIR_HIGH);
    assert(fake_clockwise[2]!=(bool)HOMING_SEEK_M3_DIR_HIGH);
    assert(!Svc_Homing_Start());
    Svc_Homing_Step();
    assert(fake_steps[0]==0); /* 未达到时间间隔不得发脉冲。 */
    fake_limits[0]=true; Tick();
    assert(fake_steps[0]==0 && fake_steps[1]==1 && fake_steps[2]==1);
    fake_limits[1]=fake_limits[2]=true; Tick();
    assert(Svc_Homing_GetState()==SVC_HOMING_BACKOFF);
    assert(fake_clockwise[0]==(bool)HOMING_SEEK_M1_DIR_HIGH);
    assert(fake_clockwise[1]==(bool)HOMING_SEEK_M2_DIR_HIGH);
    assert(fake_clockwise[2]==(bool)HOMING_SEEK_M3_DIR_HIGH);
    fake_limits[0]=fake_limits[1]=fake_limits[2]=false;
    FinishBackoff();
    assert(Svc_Homing_GetState()==SVC_HOMING_COMPLETE);
    assert(fake_steps[0]==(uint32_t)(HOMING_BACKOFF_M1_DEG*UNITS_PER_DEGREE));
    Begin();
    for(unsigned i=0;i<=HOMING_MAX_STEPS;i++) Tick();
    assert(Svc_Homing_GetState()==SVC_HOMING_FAILED);
    Begin(); fake_step_failure[0]=true; Tick();
    assert(Svc_Homing_GetState()==SVC_HOMING_FAILED);
    Begin(); fake_limits[0]=fake_limits[1]=fake_limits[2]=true; Tick(); FinishBackoff();
    assert(Svc_Homing_GetState()==SVC_HOMING_FAILED); /* 退回后开关不释放。 */
    Begin(); Tick(); unsigned before=fake_steps[0]; Svc_Homing_Cancel(); Tick();
    assert(fake_steps[0]==before);
    Svc_Homing_Cancel(); FakeRobot_SetTimeUs(UINT32_MAX-100U); assert(Svc_Homing_Start());
    Tick(); assert(fake_steps[0]==before+1); /* 微秒时间溢出后仍按间隔推进。 */
    puts("homing_service_tests: passed"); return 0;
}
