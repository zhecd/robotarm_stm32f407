/** 教学恢复窗口验收：真实闭环、队列和STEP引擎；仅替换时间/设备。
 * 跟随模型把实际发出的每个STEP换算回编码器，不直接伪造“控制器已成功”。
 * 它验证软件收敛/失败边界，不代表真实机械负载和10°行程已验收。 */
#include "motion_service.h"
#include "safety_service.h"
#include "state_service.h"
#include "fake_robot.h"
#include "robot_config.h"
#include "robot_math.h"
#include "platform_time.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
static float response;
bool __real_Dev_Joint_Step(DevJointId_t axis);
bool __wrap_Dev_Joint_Step(DevJointId_t axis)
{
    bool ok=__real_Dev_Joint_Step(axis);
    if(ok) fake_motor_deg[axis]+=(fake_clockwise[axis]?1.0f:-1.0f)*StepsToDeg(1)*response;
    return ok;
}
static ClosedLoopStatus_t Status(void)
{ ClosedLoopStatus_t s; MotionService_GetClosedLoopStatus(&s); return s; }
static void Reset(bool wrap)
{
    FakeRobot_Reset();StateService_Init();SafetyService_Init();MotionService_Init();MotionService_InitClosedLoop();
    SafetyService_MarkHomed();SafetyService_ClearAfterSuccessfulHoming();
    if(wrap) FakeRobot_SetTimeUs(((uint64_t)UINT32_MAX-500U)*1000U);
    /* 各轴目标置于软限位内部，正负10°测试不会越过M3的正向软限位。 */
    int32_t base=DegToSteps(45.0f);MotionService_AdjustTheorySteps(base,base,base);
    for(unsigned i=0;i<3;i++)fake_motor_deg[i]=StepsToDeg(base);
    MotionService_SyncClosedLoopTarget();response=0;
}
static void Advance(unsigned ms)
{
    /* TIM6=50kHz，20ms服务一次闭环；与生产时间尺度一致。 */
    for(unsigned n=0;n<ms;n++) {
        for(unsigned k=0;k<TICKS_PER_MS;k++)MotionService_OnStepTickFromISR();
        FakeRobot_AdvanceUs(1000U);
        if(n%CL_UPDATE_MS==CL_UPDATE_MS-1U) {
            MotionService_ServiceSafety();MotionService_UpdateClosedLoop();
        }
    }
}
static void Begin(unsigned axis,float error_motor)
{
    fake_motor_deg[axis]+=error_motor;
    FakeRobot_AdvanceUs((CL_SETTLE_MS+CL_COOLDOWN_LO_MS)*1000U);
    MotionService_UpdateClosedLoop();
}
static void Drain(void)
{
    unsigned n;
    for(n=0;n<100 && MotionService_IsClosedLoopRecoveryActive() && !MotionService_HasFault();n++) Advance(20);
    CHECK(n<100); CHECK(MotionService_IsIdle());
}
static void Fault(MotionFaultReason_t reason)
{
    CHECK(MotionService_GetFaultReason()==reason); CHECK(MotionService_IsIdle());
    uint32_t before[3]={fake_steps[0],fake_steps[1],fake_steps[2]};
    Advance(1000);for(unsigned i=0;i<3;i++)CHECK(fake_steps[i]==before[i]);
}
int main(int argc,char **argv)
{
    CHECK(argc==2);const char *t=argv[1];Reset(strstr(t,"wrap")!=NULL);
    if(!strncmp(t,"edge_",5) || !strncmp(t,"outside_",8)) {
        bool outside=!strncmp(t,"outside_",8);unsigned axis=(unsigned)(t[outside?8:5]-'0');
        bool neg=strstr(t,"neg")!=NULL;
        Begin(axis,(neg?-1.0f:1.0f)*(CL_RECOVERY_WINDOW_JOINT_DEG*GEAR_RATIO+(outside?.02f:0)));
        if(outside) { Fault(MOTION_FAULT_CORRECTION_RANGE);CHECK(fake_steps[axis]==0); }
        else {CHECK(!MotionService_HasFault());CHECK(Status().active);CHECK(abs(Status().last_steps[axis])==40);}
    } else if(!strcmp(t,"converge") || !strcmp(t,"partial") || !strcmp(t,"wrap_converge")) {
        response=!strcmp(t,"partial")?.7f:1.0f;Begin(0,9.0f*GEAR_RATIO);
        for(unsigned n=0;n<CL_RECOVERY_TIMEOUT_MS/20U && !MotionService_HasFault();n++) {
            Advance(20);ClosedLoopStatus_t s=Status();
            for(unsigned i=0;i<3;i++)CHECK(abs(s.last_steps[i])<=40);
            if(MotionService_IsIdle() && !s.active && (s.decision_mask&1) && fabsf(s.error_motor_deg[0])<=CL_DEADBAND_DEG)break;
        }
        CHECK(!MotionService_HasFault());CHECK(fabsf(fake_motor_deg[0]-45.0f)<=CL_DEADBAND_DEG+.01f);
        CHECK(Status().accepted_rounds[0]>1);int32_t a,b,c;MotionService_GetTheorySteps(&a,&b,&c);
        CHECK(a==DegToSteps(45) && b==a && c==a);
    } else if(!strcmp(t,"deadband")) {
        Begin(0,CL_DEADBAND_DEG);Advance(1000);CHECK(!MotionService_HasFault());CHECK(fake_steps[0]==0);
    } else if(!strcmp(t,"stalled") || !strcmp(t,"small_stalled")) {
        Begin(0,!strcmp(t,"small_stalled")?4.0f:20.0f);Advance(4000);Fault(MOTION_FAULT_CORRECTION_STALLED);
        CHECK(Status().accepted_rounds[0]==CL_RECOVERY_NO_PROGRESS_LIMIT);
    } else if(!strcmp(t,"divergence")) {
        response=-1.0f;Begin(0,10.0f);Advance(3000);Fault(MOTION_FAULT_CONTROL_DIVERGENCE);
    } else if(!strcmp(t,"arrival_before_stall")) {
        Begin(0,10);Drain();Advance(CL_SETTLE_MS+10);Drain();
        fake_motor_deg[0]=45.0f+CL_DEADBAND_DEG;Advance(600);
        CHECK(!MotionService_HasFault());CHECK(!Status().active);
        /* 到位后允许新扰动建立新的会话，不能继承前两轮无改善计数。 */
        Begin(0,10);CHECK(!MotionService_HasFault());
    } else if(!strcmp(t,"timeout_busy") || !strcmp(t,"timeout_settle") || !strcmp(t,"wrap_timeout")) {
        Begin(0,10);uint32_t started=PlatformTime_NowMs();
        if(!strcmp(t,"timeout_settle"))Drain();
        FakeRobot_AdvanceUs((CL_RECOVERY_TIMEOUT_MS-(uint32_t)(PlatformTime_NowMs()-started)-1U)*1000U);
        MotionService_UpdateClosedLoop();CHECK(!MotionService_HasFault());
        FakeRobot_AdvanceUs(1000);MotionService_UpdateClosedLoop();Fault(MOTION_FAULT_CORRECTION_TIMEOUT);
    } else if(!strcmp(t,"raw_window") || !strcmp(t,"busy_window") || !strcmp(t,"settle_window")) {
        if(!strcmp(t,"raw_window")) {Advance(600);}
        else {Begin(0,10);if(!strcmp(t,"settle_window"))Drain();}
        fake_motor_deg[0]=45.0f+CL_RECOVERY_WINDOW_JOINT_DEG*GEAR_RATIO+.02f;
        MotionService_UpdateClosedLoop();Fault(MOTION_FAULT_CORRECTION_RANGE);
    } else if(!strcmp(t,"sync_resets") || !strcmp(t,"disable_resets")) {
        Begin(0,10);Drain();
        if(!strcmp(t,"sync_resets"))MotionService_SyncClosedLoopTarget();
        else {MotionService_SetClosedLoopAxisEnabled(0,false);MotionService_SetClosedLoopAxisEnabled(0,true);}
        FakeRobot_AdvanceUs((CL_RECOVERY_TIMEOUT_MS+1U)*1000U);MotionService_UpdateClosedLoop();
        CHECK(!MotionService_HasFault());CHECK(Status().active);
    } else if(!strcmp(t,"encoder")) {
        Begin(0,10);fake_encoder_failure[0]=true;Advance(100);Fault(MOTION_FAULT_ENCODER);
    } else if(!strcmp(t,"soft_limit")) {
        Begin(0,10);fake_motor_deg[0]=500;MotionService_UpdateClosedLoop();Fault(MOTION_FAULT_SOFT_LIMIT);
    } else if(!strcmp(t,"limit")) {
        Begin(0,10);MotionService_SetLimitMonitoring(true);
        fake_limits[0]=true;MotionService_NotifyLimitSwitch(MOTION_LIMIT_M1);
        Advance(100);Fault(MOTION_FAULT_LIMIT_SWITCH);
    } else if(!strcmp(t,"normal_busy")) {
        MotionFrame_t f={.delta_m1=1,.total_ticks=100000};
        CHECK(MotionService_SubmitFrame(&f));fake_motor_deg[0]=120;
        MotionService_UpdateClosedLoop();CHECK(!MotionService_HasFault());CHECK(Status().accepted_rounds[0]==0);
    } else CHECK(false);
    printf("closed_loop_window %s passed\n",t);return 0;
}
