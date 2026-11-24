/** @file svc_homing_sequence.c
 * @brief 将纯回零算法的请求应用到逻辑关节、限位和保持输出。
 * @details 1ms ISR只处理GPIO及有限输出操作；主循环提供心跳与状态查询。
 * GPIO模式的rearm仅确认释放，不重新打开EXTI。标定与READY提交由App编排，
 * 本服务不读取I2C、不处理串口；所有共享状态访问使用短临界区。
 */
#include "service/svc_homing_sequence.h"
#include "device/dev_joint.h"
#include "device/dev_limit_switch.h"
#include "platform_time.h"
#include "platform_critical.h"
#include "limit_irq_guard.h"
#include "robot_config.h"

static HomeSequence s_home;
#ifndef ROBOTARM_PARALLEL_HOMING
#define ROBOTARM_PARALLEL_HOMING 0
#endif
static bool s_configured,s_committed;
static bool s_calibration_hold;
static uint32_t s_main_seen;
static const HomeConfig s_config={
    .period_ms=HOMING_SEQUENCE_SEEK_PERIOD_MS,.seek_timeout_ms=0U,.total_timeout_ms=220000U,
    .backoff={(uint32_t)(HOMING_BACKOFF_M1_DEG*UNITS_PER_DEGREE),
              (uint32_t)(HOMING_SEQUENCE_BACKOFF_M2_DEG*UNITS_PER_DEGREE),
              (uint32_t)(HOMING_SEQUENCE_BACKOFF_M3_DEG*UNITS_PER_DEGREE)},
    .seek_dir=(HOMING_SEEK_M1_DIR_HIGH<<0)|(HOMING_SEEK_M2_DIR_HIGH<<1)|(HOMING_SEEK_M3_DIR_HIGH<<2),
    /* 寻限位/退回默认均1000微步/秒；单轴截止关闭，整轮220秒截止保留。 */
    .backoff_period_ms=HOMING_SEQUENCE_BACKOFF_PERIOD_MS,
#if ROBOTARM_LIMIT_STATE_MODE
    .qualify_unexpected=true
#else
    .qualify_unexpected=false
#endif
};
static uint8_t ReadLow(void)
{
    uint8_t low=0;
    for(uint8_t a=0;a<3;a++)if(Dev_LimitSwitch_IsTriggered((DevJointId_t)a))low|=(uint8_t)(1U<<a);
    return low;
}
static void StopJoints(void)
{
    Dev_Joint_EnableAll(false);
    for(uint8_t a=0;a<3;a++)(void)Dev_Joint_WriteStepLevel((DevJointId_t)a,false);
}
/** 调用者持短临界区；先停再屏蔽，再处理经稳定释放确认的单轴恢复。 */
static void ApplyLocked(void)
{
    if(LimitIrqGuard_GetLatchMask())HomeSequence_Fail(&s_home,HOME_IRQ);
    if((s_home.mask_request && !s_home.parallel) || !s_home.enable || s_home.phase==HOME_FAILED)StopJoints();
    for(uint8_t a=0;a<3;a++)if(s_home.mask_request&(1U<<a))
        Dev_LimitSwitch_MaskInterrupt((DevJointId_t)a);
    s_home.mask_request=0;
    for(uint8_t a=0;a<3;a++)if(s_home.rearm_request&(1U<<a)){
        bool ok=!LimitIrqGuard_GetLatchMask() && Dev_LimitSwitch_TryRearmInterrupt((DevJointId_t)a);
        HomeSequence_AckRearm(&s_home,a,ok);
    }
    if(s_home.phase==HOME_FAILED || !s_home.enable){StopJoints();return;}
    /* 并行模式按三轴位图写DIR/STEP，旧对照只写当前轴；共用EN始终作用于三路。
     * 每tick先写低，再更新DIR/EN后产生各轴最多一次上升沿，不补发欠步。
     * 1ms周期连续发步时，低电平由这些GPIO操作间隔形成；不能再声称低电平
     * 固定保持1ms。端口写入顺序由电脑测试覆盖，真实脉宽仍需仪器实测。 */
    bool ok=true;
    for(uint8_t a=0;a<3;a++)ok=Dev_Joint_WriteStepLevel((DevJointId_t)a,false)&&ok;
    if(s_home.parallel){
        for(uint8_t a=0;a<3;a++)ok=Dev_Joint_SetDirectionLevel((DevJointId_t)a,(s_home.dir_mask&(1U<<a))!=0)&&ok;
    }else ok=Dev_Joint_SetDirectionLevel((DevJointId_t)s_home.axis,s_home.dir_high)&&ok;
    if(!ok || LimitIrqGuard_GetLatchMask()){
        HomeSequence_Fail(&s_home,LimitIrqGuard_GetLatchMask()?HOME_IRQ:HOME_PORT);StopJoints();return;
    }
    Dev_Joint_EnableAll(true);
    uint8_t pulses=s_home.parallel?s_home.step_mask:s_home.high?(uint8_t)(1U<<s_home.axis):0U;
    for(uint8_t a=0;a<3;a++)if((pulses&(1U<<a)) && !Dev_Joint_WriteStepLevel((DevJointId_t)a,true)){
        HomeSequence_Fail(&s_home,HOME_PORT);StopJoints();break;
    }
}
void Svc_HomingSequence_OnLimitsConfigured(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    if(!s_configured){HomeSequence_Init(&s_home,&s_config);s_home.parallel=ROBOTARM_PARALLEL_HOMING;
        s_configured=true;StopJoints();}
    PlatformCritical_Exit(key);
}
void Svc_HomingSequence_OnTickFromISR(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    if(s_configured && !s_committed){
        uint32_t now=PlatformTime_NowMs();
        if(LimitIrqGuard_GetLatchMask())HomeSequence_Fail(&s_home,HOME_IRQ);
        if(s_home.started && (s_home.phase!=HOME_DONE || s_calibration_hold) &&
           (uint32_t)(now-s_main_seen)>=250U)HomeSequence_Fail(&s_home,HOME_MAIN_STALLED);
        if(s_home.phase==HOME_DONE && ReadLow())HomeSequence_Fail(&s_home,HOME_UNEXPECTED);
        HomeSequence_Tick(&s_home,now,ReadLow());
        if(s_calibration_hold && s_home.phase==HOME_DONE){
            if(!LimitIrqGuard_GetLatchMask())Dev_Joint_EnableAll(true);
            else{HomeSequence_Fail(&s_home,HOME_IRQ);StopJoints();}
        }else ApplyLocked();
    }
    PlatformCritical_Exit(key);
}
bool Svc_HomingSequence_OnLimitFromISR(uint8_t axis)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();bool handled=false;
    if(s_configured && !s_committed){
        handled=HomeSequence_OnEdge(&s_home,axis,PlatformTime_NowMs(),ReadLow());
        if(handled)ApplyLocked();
    }
    PlatformCritical_Exit(key);return handled;
}
bool Svc_HomingSequence_PrepareStartup(bool recovery)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    bool ok=s_configured && !LimitIrqGuard_GetLatchMask();
    if(ok && recovery){
        StopJoints();uint8_t old=s_home.owned;
        HomeSequence_PrepareRecovery(&s_home,&s_config,old,PlatformTime_NowMs());
        s_home.parallel=ROBOTARM_PARALLEL_HOMING;
        s_committed=false;s_calibration_hold=false;
        HomeSequence_Tick(&s_home,PlatformTime_NowMs(),ReadLow());ApplyLocked();
    }
    if(!ok && s_configured)HomeSequence_Fail(&s_home,LimitIrqGuard_GetLatchMask()?HOME_IRQ:HOME_PORT);
    bool accepted=ok && s_home.phase!=HOME_FAILED;
    PlatformCritical_Exit(key);return accepted;
}
bool Svc_HomingSequence_BootReady(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    bool ok=s_configured && !LimitIrqGuard_GetLatchMask() && HomeSequence_BootReady(&s_home,ReadLow());
    PlatformCritical_Exit(key);return ok;
}
bool Svc_HomingSequence_Start(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();s_main_seen=PlatformTime_NowMs();
    bool ok=s_configured && !LimitIrqGuard_GetLatchMask() && HomeSequence_Start(&s_home,s_main_seen,ReadLow());
    if(ok)ApplyLocked();
    PlatformCritical_Exit(key);return ok;
}
void Svc_HomingSequence_MainHeartbeat(void)
{PlatformCriticalState_t k=PlatformCritical_Enter();s_main_seen=PlatformTime_NowMs();PlatformCritical_Exit(k);}
void Svc_HomingSequence_Cancel(void)
{
    PlatformCriticalState_t k=PlatformCritical_Enter();
    HomeSequence_Fail(&s_home,LimitIrqGuard_GetLatchMask()?HOME_IRQ:HOME_CANCELLED);
    StopJoints();PlatformCritical_Exit(k);
}
bool Svc_HomingSequence_CommitReady(void)
{
    PlatformCriticalState_t k=PlatformCritical_Enter();
    bool ok=s_configured && s_calibration_hold && s_home.phase==HOME_DONE && s_home.completed==7U &&
        s_home.rearmed==7U && !s_home.owned && !s_home.rearm_request &&
        !ReadLow() && !LimitIrqGuard_GetLatchMask();
    if(ok)s_committed=true;
    PlatformCritical_Exit(k);return ok;
}
bool Svc_HomingSequence_HoldForCalibration(void)
{
    PlatformCriticalState_t k=PlatformCritical_Enter();
    bool ok=s_configured && s_home.phase==HOME_DONE && s_home.completed==7U &&
        s_home.rearmed==7U && !s_home.owned && !ReadLow() && !LimitIrqGuard_GetLatchMask();
    if(ok){s_calibration_hold=true;s_main_seen=PlatformTime_NowMs();Dev_Joint_EnableAll(true);}
    PlatformCritical_Exit(k);return ok;
}
bool Svc_HomingSequence_HasFailure(void)
{PlatformCriticalState_t k=PlatformCritical_Enter();bool f=s_configured && s_home.phase==HOME_FAILED;PlatformCritical_Exit(k);return f;}
bool Svc_HomingSequence_RuntimeOwnsLimits(void)
{PlatformCriticalState_t k=PlatformCritical_Enter();bool ready=s_committed;PlatformCritical_Exit(k);return ready;}
bool Svc_HomingSequence_ValidateHandoff(void)
{
    PlatformCriticalState_t k=PlatformCritical_Enter();
    if(s_committed && (ReadLow() || LimitIrqGuard_GetLatchMask())){
        HomeSequence_Fail(&s_home,LimitIrqGuard_GetLatchMask()?HOME_IRQ:HOME_UNEXPECTED);StopJoints();
    }
    bool ok=s_home.phase!=HOME_FAILED;PlatformCritical_Exit(k);return ok;
}
void Svc_HomingSequence_GetStatus(SvcHomingSequenceStatus *out)
{
    if(!out)return;
    PlatformCriticalState_t k=PlatformCritical_Enter();
    bool ready=s_configured && !LimitIrqGuard_GetLatchMask() && HomeSequence_BootReady(&s_home,ReadLow());
    *out=(SvcHomingSequenceStatus){s_home,s_configured,s_committed,ready};PlatformCritical_Exit(k);
}
