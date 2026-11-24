/** 闭环与PS2补齐验收：调用真实业务实现，只替换设备、时间和手柄数据。 */
#include "app/app_teleop.h"
#include "command_service.h"
#include "motion_service.h"
#include "state_service.h"
#include "safety_service.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include "robot_config.h"
#include "robot_math.h"
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
static unsigned input_reads;
static int32_t signed_steps[3];
bool __real_Dev_Joint_Step(DevJointId_t axis);
bool __wrap_Dev_Joint_Step(DevJointId_t axis)
{
    bool ok=__real_Dev_Joint_Step(axis);
    if(ok)signed_steps[axis]+=fake_clockwise[axis]?1:-1;
    return ok;
}
bool __real_AppHardware_ReadTeleopInput(AppTeleopInput_t *out);
bool __wrap_AppHardware_ReadTeleopInput(AppTeleopInput_t *out)
{
    input_reads++;return __real_AppHardware_ReadTeleopInput(out);
}
static void Reset(void)
{
    FakeRobot_Reset();FakeHardware_Reset();StateService_Init();SafetyService_Init();
    MotionService_Init();MotionService_InitClosedLoop();
    assert(CommandService_Init(HOMEPOSE_X_MM,HOMEPOSE_Y_MM,HOMEPOSE_Z_MM)==ERR_OK);
    SafetyService_MarkHomed();SafetyService_ClearAfterSuccessfulHoming();
    MotionService_SyncClosedLoopTarget();
    App_Teleop_SetMode(SYS_MODE_GCODE);App_Teleop_Init();
    fake_ps2_valid=true;input_reads=0;memset(signed_steps,0,sizeof(signed_steps));
}
static void Poll(void)
{
    FakeRobot_AdvanceUs(PS2_POLL_INTERVAL_MS*1000U);App_Teleop_Task();
}
static void Neutral(void)
{
    fake_ps2_input=(AppTeleopInput_t){128,128,128,false,false};Poll();
}
static void NoActions(void)
{
    assert(MotionService_GetQueueCount()==0 && !MotionService_IsRunning());
    assert(fake_gripper_open_count==0 && fake_gripper_close_count==0);
}
static void BeginCorrection(float motor_deg)
{
    fake_motor_deg[0]=motor_deg;
    FakeRobot_AdvanceUs((CL_SETTLE_MS+CL_COOLDOWN_LO_MS)*1000U);
    MotionService_UpdateClosedLoop();assert(MotionService_IsClosedLoopRecoveryActive());
}
/* 保持输入角度不变，模拟设备未跟随；不是机械动力学模型。 */
static void DrainCorrection(void)
{
    unsigned n;
    for(n=0;n<4000 && MotionService_IsClosedLoopRecoveryActive();n++) {
        for(unsigned k=0;k<1000;k++)MotionService_OnStepTickFromISR();
        FakeRobot_AdvanceUs(CL_UPDATE_MS*1000U);MotionService_UpdateClosedLoop();
    }
    assert(n<4000 && MotionService_IsIdle());
}
int main(int argc,char **argv)
{
    assert(argc==2);Reset();const char *test=argv[1];
    if(!strcmp(test,"acquire_success")) {
        App_Teleop_SetMode(SYS_MODE_PS2);fake_ps2_valid=false;
        for(unsigned i=0;i<60;i++){Poll();assert(!MotionService_HasFault());NoActions();}
        fake_ps2_valid=true;Neutral();NoActions();
        fake_ps2_input.left_x=255;Poll();assert(MotionService_GetQueueCount()>0);
    } else if(!strcmp(test,"acquire_timeout")) {
        App_Teleop_SetMode(SYS_MODE_PS2);fake_ps2_valid=false;
        for(unsigned i=0;i<PS2_INPUT_ACQUIRE_TIMEOUT_MS/PS2_POLL_INTERVAL_MS-1U;i++) {
            Poll();assert(!MotionService_HasFault());NoActions();
        }
        Poll();assert(MotionService_GetFaultReason()==MOTION_FAULT_INPUT_LOST);NoActions();
    } else if(!strcmp(test,"acquire_then_loss")) {
        App_Teleop_SetMode(SYS_MODE_PS2);Neutral();fake_ps2_valid=false;Poll();
        assert(MotionService_GetFaultReason()==MOTION_FAULT_INPUT_LOST);NoActions();
    } else if(!strcmp(test,"held_then_loss")) {
        App_Teleop_SetMode(SYS_MODE_PS2);fake_ps2_input.left_x=255;Poll();NoActions();
        fake_ps2_valid=false;Poll();assert(MotionService_GetFaultReason()==MOTION_FAULT_INPUT_LOST);
    } else if(!strcmp(test,"conflict_then_loss")) {
        App_Teleop_SetMode(SYS_MODE_PS2);Neutral();
        fake_ps2_input.cross_pressed=true;fake_ps2_input.square_pressed=true;Poll();NoActions();
        fake_ps2_valid=false;Poll();assert(MotionService_GetFaultReason()==MOTION_FAULT_INPUT_LOST);
    } else if(!strcmp(test,"acquire_queued")) {
        App_Teleop_SetMode(SYS_MODE_PS2);fake_ps2_valid=false;
        assert(CommandService_RunTeleopStep(.9f,0,0)==ERR_OK);Poll();
        assert(MotionService_GetFaultReason()==MOTION_FAULT_INPUT_LOST);NoActions();
    } else if(!strcmp(test,"entry_stick")) {
        App_Teleop_SetMode(SYS_MODE_PS2);fake_ps2_input.left_x=255;
        Poll();Poll();NoActions();Neutral();NoActions();
        fake_ps2_input.left_x=255;Poll();assert(MotionService_GetQueueCount()>0);
    } else if(!strcmp(test,"entry_button")) {
        App_Teleop_SetMode(SYS_MODE_PS2);fake_ps2_input.cross_pressed=true;
        Poll();Poll();NoActions();Neutral();NoActions();
        fake_ps2_input.cross_pressed=true;Poll();Poll();assert(fake_gripper_close_count==1);
    } else if(!strcmp(test,"reentry")) {
        App_Teleop_SetMode(SYS_MODE_PS2);Neutral();
        App_Teleop_ToggleMode();fake_ps2_input.square_pressed=true;Poll();
        App_Teleop_ToggleMode();Poll();NoActions();
        Neutral();fake_ps2_input.square_pressed=true;Poll();assert(fake_gripper_open_count==1);
    } else if(!strcmp(test,"reinitialize")) {
        App_Teleop_SetMode(SYS_MODE_PS2);Neutral();
        /* 真实回零完成也调用Init，回零期间按住的控制不能直接生效。 */
        App_Teleop_Init();fake_ps2_input.right_y=0;Poll();NoActions();
        Neutral();fake_ps2_input.left_x=255;Poll();assert(MotionService_GetQueueCount()>0);
    } else if(!strcmp(test,"gcode_no_poll")) {
        fake_ps2_valid=false;for(unsigned i=0;i<20;i++)Poll();
        assert(input_reads==0 && !MotionService_HasFault());NoActions();
    } else if(!strcmp(test,"conflicting_buttons")) {
        App_Teleop_SetMode(SYS_MODE_PS2);Neutral();
        fake_ps2_input.cross_pressed=true;fake_ps2_input.square_pressed=true;Poll();NoActions();
        fake_ps2_input.square_pressed=false;Poll();NoActions();
        Neutral();fake_ps2_input.cross_pressed=true;Poll();assert(fake_gripper_close_count==1);
    } else if(!strcmp(test,"alternating_buttons")) {
        App_Teleop_SetMode(SYS_MODE_PS2);Neutral();
        for(unsigned i=0;i<8;i++) {
            fake_ps2_input.square_pressed=i%2U==0;
            fake_ps2_input.cross_pressed=i%2U!=0;
            Poll();Poll(); /* 按住不重复；下一目标可以直接替换，无需等600ms。 */
            assert(fake_gripper_open_count==(i+2U)/2U);
            assert(fake_gripper_close_count==(i+1U)/2U);
        }
        fake_ps2_valid=false;Poll();
        assert(MotionService_HasFault());
        fake_ps2_valid=true;Poll();
        assert(fake_gripper_open_count==4 && fake_gripper_close_count==4);
    } else if(!strcmp(test,"deadband")) {
        fake_motor_deg[0]=CL_DEADBAND_DEG;
        for(unsigned i=0;i<30;i++) {
            FakeRobot_AdvanceUs(CL_UPDATE_MS*1000U);MotionService_UpdateClosedLoop();
        }
        NoActions();assert(!MotionService_IsClosedLoopRecoveryActive());
    } else if(!strcmp(test,"direction_positive") || !strcmp(test,"direction_negative")) {
        float angle=!strcmp(test,"direction_positive")?10.0f:-10.0f;
        BeginCorrection(angle);DrainCorrection();
        assert(fake_steps[0]>0 && fake_steps[1]==0 && fake_steps[2]==0);
        /* 零步帧也会写DIR；必须对实际STEP计方向，不能看最后的静态DIR。 */
        assert((signed_steps[0]>0)==(angle<0));
        assert((uint32_t)abs(signed_steps[0])==fake_steps[0]);
        int32_t a,b,c;MotionService_GetTheorySteps(&a,&b,&c);assert(a==0 && b==0 && c==0);
        fake_motor_deg[0]=0;FakeRobot_AdvanceUs((CL_SETTLE_MS+1U)*1000U);MotionService_UpdateClosedLoop();
        assert(!MotionService_IsClosedLoopRecoveryActive() && !MotionService_HasFault());
    } else if(!strcmp(test,"bounded")) {
        BeginCorrection(40.0f);DrainCorrection();
        assert(fake_steps[0]==(uint32_t)abs(DegToSteps(CL_RECOVERY_ROUND_JOINT_DEG*GEAR_RATIO)));
    } else if(!strcmp(test,"no_progress")) {
        BeginCorrection(10.0f);
        for(unsigned i=0;i<CL_RECOVERY_NO_PROGRESS_LIMIT;i++) {
            DrainCorrection();FakeRobot_AdvanceUs((CL_SETTLE_MS+1U)*1000U);MotionService_UpdateClosedLoop();
        }
        assert(MotionService_GetFaultReason()==MOTION_FAULT_CORRECTION_STALLED);
        assert(MotionService_IsIdle() && !MotionService_IsClosedLoopRecoveryActive());
    } else if(!strcmp(test,"divergence")) {
        BeginCorrection(10.0f);
        for(unsigned i=0;i<CL_DIVERGENCE_LIMIT;i++) {
            DrainCorrection();fake_motor_deg[0]+=5.0f;
            FakeRobot_AdvanceUs((CL_SETTLE_MS+1U)*1000U);MotionService_UpdateClosedLoop();
        }
        assert(MotionService_GetFaultReason()==MOTION_FAULT_CONTROL_DIVERGENCE);
        assert(MotionService_IsIdle());
    } else if(!strcmp(test,"small_correction")) {
        BeginCorrection(4.0f);DrainCorrection();
        assert(signed_steps[0]<0 && fake_steps[0]<=(uint32_t)abs(DegToSteps(CL_MAX_CORRECTION_DEG)));
        int32_t a,b,c;MotionService_GetTheorySteps(&a,&b,&c);assert(a==0 && b==0 && c==0);
    } else if(!strcmp(test,"stop_recovery")) {
        BeginCorrection(10.0f);MotionService_OnStepTickFromISR();
        MotionService_AbortForFault(MOTION_FAULT_HOST_STOP);
        assert(!MotionService_IsClosedLoopRecoveryActive());
        uint32_t before=fake_steps[0];
        for(unsigned i=0;i<2000;i++) { MotionService_OnStepTickFromISR();MotionService_UpdateClosedLoop(); }
        assert(fake_steps[0]==before && MotionService_IsIdle());
        assert(MotionService_GetFaultReason()==MOTION_FAULT_HOST_STOP);
    } else { assert(false); }
    printf("closed_loop_teleop_completion (%s): passed\n",test);return 0;
}
