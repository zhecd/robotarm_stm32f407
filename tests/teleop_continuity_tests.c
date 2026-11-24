/** 链接真实App/命令/规划/插补/安全实现，只替换时间与设备端口。
 * Submit包装器仅记录已受理帧，仍调用真实提交函数；STEP由真实ISR输出。
 * 覆盖有界预填、帧边界连续性、松杆排空及故障撤销，不把它当声学测试。 */
#include "app/app_teleop.h"
#include "command_service.h"
#include "motion_service.h"
#include "safety_service.h"
#include "state_service.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include "robot_config.h"
#include "robot_math.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static MotionFrame_t accepted[128];
static unsigned submitted;
bool __real_MotionService_SubmitFrame(const MotionFrame_t *frame);
bool __wrap_MotionService_SubmitFrame(const MotionFrame_t *frame)
{
    bool ok=__real_MotionService_SubmitFrame(frame);
    if(ok) { assert(submitted<128);accepted[submitted++]=*frame; }
    return ok;
}
static void Reset(void)
{
    FakeRobot_Reset();FakeHardware_Reset();StateService_Init();SafetyService_Init();
    MotionService_Init();MotionService_InitClosedLoop();
    assert(CommandService_Init(HOMEPOSE_X_MM,HOMEPOSE_Y_MM,HOMEPOSE_Z_MM)==ERR_OK);
    SafetyService_MarkHomed();SafetyService_ClearAfterSuccessfulHoming();
    MotionService_SyncClosedLoopTarget();
    App_Teleop_SetMode(SYS_MODE_GCODE);App_Teleop_Init();submitted=0;
    fake_ps2_valid=true;
}
static void Ticks(uint32_t n)
{
    while(n--) { FakeRobot_AdvanceUs(1000U/TICKS_PER_MS);MotionService_OnStepTickFromISR(); }
}
static void Poll(void)
{
    FakeRobot_AdvanceUs(PS2_POLL_INTERVAL_MS*1000U);App_Teleop_Task();
}
static void Arm(void)
{
    App_Teleop_SetMode(SYS_MODE_PS2);
    fake_ps2_input=(AppTeleopInput_t){128,128,128,false,false};Poll();
}
static void CheckCounts(void)
{
    uint32_t a=0,b=0,c=0;
    for(unsigned i=0;i<submitted;i++) {
        a+=(uint32_t)abs(accepted[i].delta_m1);
        b+=(uint32_t)abs(accepted[i].delta_m2);
        c+=(uint32_t)abs(accepted[i].delta_m3);
    }
    assert(fake_steps[0]==a && fake_steps[1]==b && fake_steps[2]==c);
}
int main(int argc,char **argv)
{
    assert(argc==2);Reset();const char *test=argv[1];
    if(!strcmp(test,"continuity")) {
        assert(CommandService_RunTeleopStep(.9f,0,0)==ERR_OK);
        for(unsigned i=0;i<40;i++) {
            /* 消耗当前帧前半段，再预填；边界到下一次前台轮询前仍须有STEP。 */
            uint32_t duration=accepted[i].total_ticks;
            Ticks(duration/2U-(i==0?0U:1U));
            assert(MotionService_IsRunning());
            assert(CommandService_RunTeleopStep(.9f,0,0)==ERR_OK);
            assert(MotionService_GetQueueCount()==1);
            Ticks(duration-duration/2U);
            assert(!MotionService_IsRunning() && !MotionService_IsIdle());
            Ticks(1);assert(MotionService_IsRunning() && MotionService_GetQueueCount()==0);
        }
        while(!MotionService_IsIdle())Ticks(1);
        CheckCounts();assert(!MotionService_HasFault());
    } else if(!strcmp(test,"bound") || !strcmp(test,"target")) {
        int32_t a0,b0,c0;MotionService_GetTheorySteps(&a0,&b0,&c0);
        assert(CommandService_RunTeleopStep(.9f,0,0)==ERR_OK);Ticks(1);
        assert(CommandService_RunTeleopStep(.9f,0,0)==ERR_OK);
        CommandServiceStatus_t before,after;CommandService_GetStatus(&before);
        for(unsigned i=0;i<100;i++)assert(CommandService_RunTeleopStep(.9f,0,0)==ERR_BUSY);
        CommandService_GetStatus(&after);
        assert(before.generation==after.generation && before.planned_x_mm==after.planned_x_mm);
        assert(submitted==2 && MotionService_GetQueueCount()==1);
        Ticks(accepted[0].total_ticks+accepted[1].total_ticks-1U);
        assert(MotionService_IsIdle());CheckCounts();
        int32_t a,b,c;MotionService_GetTheorySteps(&a,&b,&c);
        assert(a-a0==accepted[0].delta_m1+accepted[1].delta_m1);
        assert(b-b0==accepted[0].delta_m2+accepted[1].delta_m2);
        assert(c-c0==accepted[0].delta_m3+accepted[1].delta_m3);
        if(!strcmp(test,"target")) {
            /* 设备端口返回与理论相同的反馈，静置不能追逐上一个短帧的旧目标。 */
            fake_motor_deg[0]=StepsToDeg(a);
            fake_motor_deg[1]=StepsToDeg(b);
            fake_motor_deg[2]=StepsToDeg(c);
            for(unsigned i=0;i<20;i++) {
                FakeRobot_AdvanceUs((CL_SETTLE_MS+20U)*1000U);MotionService_UpdateClosedLoop();
                assert(!MotionService_IsClosedLoopRecoveryActive() && MotionService_IsIdle());
            }
        }
    } else if(!strcmp(test,"neutral") || !strcmp(test,"loss") || !strcmp(test,"stop")) {
        Arm();fake_ps2_input.left_x=255;Poll();Ticks(1);Poll();
        assert(submitted==2 && MotionService_GetQueueCount()==1);
        if(!strcmp(test,"neutral")) {
            fake_ps2_input.left_x=128;Poll();
            assert(submitted==2 && MotionService_GetQueueCount()==1);
            Ticks(accepted[0].total_ticks+accepted[1].total_ticks-1U);
            assert(MotionService_IsIdle());CheckCounts();Poll();assert(submitted==2);
        } else {
            uint32_t before[3]={fake_steps[0],fake_steps[1],fake_steps[2]};
            if(!strcmp(test,"loss")) { fake_ps2_valid=false;Poll(); }
            else MotionService_AbortForFault(MOTION_FAULT_HOST_STOP);
            Ticks(10000);
            assert(MotionService_HasFault() && MotionService_IsIdle());
            /* 这里直接调用运动层；AppRuntime随后才同步全局安全故障。
             * 当前层自己的许可必须已经拒绝，不能等待下一轮主循环才停STEP。 */
            assert(!MotionService_IsActionAllowed());
            assert(fake_steps[0]==before[0] && fake_steps[1]==before[1] && fake_steps[2]==before[2]);
            assert(CommandService_RunTeleopStep(.9f,0,0)==ERR_BUSY);
        }
    } else if(!strcmp(test,"gcode")) {
        GCodeFrame_t move={.type=GCMD_G1,.has_x=true,.x=5,.has_f=true,.f=120};
        ErrorCode_t result=CommandService_RunGCode(&move);
        assert(result==ERR_PENDING || result==ERR_OK);
        for(unsigned i=0;i<100;i++)CommandService_Service();
        assert(!MotionService_IsIdle());
        CommandServiceStatus_t before,after;CommandService_GetStatus(&before);
        assert(CommandService_RunTeleopStep(.9f,0,0)==ERR_BUSY);
        CommandService_GetStatus(&after);assert(before.generation==after.generation);
    } else if(!strcmp(test,"invalid")) {
        assert(CommandService_RunTeleopStep(.9f,0,0)==ERR_OK);Ticks(1);
        CommandServiceStatus_t before,after;CommandService_GetStatus(&before);
        assert(CommandService_RunTeleopStep(NAN,0,0)!=ERR_OK);
        assert(CommandService_RunTeleopStep(1e8f,0,0)!=ERR_OK);
        CommandService_GetStatus(&after);
        assert(before.generation==after.generation && submitted==1 && MotionService_GetQueueCount()==0);
    } else { assert(false); }
    printf("teleop_continuity (%s): passed\n",test);return 0;
}
