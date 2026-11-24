/** 真实App/协议/安全服务/逐轴回零控制器/ISR集成；只替换设备与时间端口。
 * 正常退回使用4000/1240/400步，模拟GPIO在退回2步后释放，不模拟机械负载。 */
#include "app_init.h"
#include "app/app_teleop.h"
#include "app_isr_adapter.h"
#include "service/svc_homing_sequence.h"
#include "command_service.h"
#include "motion_service.h"
#include "robot_config.h"
#include "device/dev_limit_switch.h"
#include "limit_irq_guard.h"
#include "safety_service.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned rearms[3],masks[3];
static bool refuse_rearm;
static const char *mode;
static bool injected;
static void Edge(uint8_t a);
void __real_AppHardware_EnableJoints(bool enabled);
void __wrap_AppHardware_EnableJoints(bool enabled)
{
    __real_AppHardware_EnableJoints(enabled);
    SvcHomingSequenceStatus s;Svc_HomingSequence_GetStatus(&s);
    if(enabled && s.committed && !injected){
        if(!strcmp(mode,"handoff_irq")){
            injected=true;for(unsigned i=0;i<256;i++)Edge(1);
        }else if(!strcmp(mode,"handoff_limit")){
            injected=true;fake_limits[1]=true;Edge(1);
        }
    }
}
ErrorCode_t __real_Dev_Joint_SetFeedbackZero(DevJointId_t a);
ErrorCode_t __wrap_Dev_Joint_SetFeedbackZero(DevJointId_t a)
{
    ErrorCode_t result=__real_Dev_Joint_SetFeedbackZero(a);
    if(!injected && !strcmp(mode,"calibration_irq")){
        injected=true;for(unsigned i=0;i<256;i++)Edge(1);
    }
    return result;
}
void Dev_LimitSwitch_MaskInterrupt(DevJointId_t a)
{
    assert((unsigned)a<3 && !fake_joints_enabled);
    assert(!fake_wave_level[0] && !fake_wave_level[1] && !fake_wave_level[2]);
    fake_limit_irq_mask|=(uint8_t)(1U<<a);masks[a]++;
}
bool Dev_LimitSwitch_TryRearmInterrupt(DevJointId_t a)
{
    if((unsigned)a>=3 || fake_limits[a] || refuse_rearm || LimitIrqGuard_GetLatchMask())return false;
    fake_limit_irq_mask&=(uint8_t)~(1U<<a);rearms[a]++;return true;
}
static void Edge(uint8_t a)
{if(!(fake_limit_irq_mask&(1U<<a)))App_ISR_OnGpioEdge((uint16_t)(M1_STOP_Pin<<a));}
static SvcHomingSequenceStatus Status(void)
{SvcHomingSequenceStatus s;Svc_HomingSequence_GetStatus(&s);return s;}
static void Send(const char *line)
{FakeHardware_ClearTx();FakeHardware_SendLine(line);App_RunOnce();}
static void CheckClosedLoopReference(const char *expected)
{
    Send("M411");
    if(!strstr(fake_tx,expected) || !strstr(fake_tx,"TARGET_MOTOR:0,0,0") || !strstr(fake_tx,"ROUNDS:0,0,0")){
        fprintf(stderr,"closed-loop reference expected %s, got %s\n",expected,fake_tx);exit(2);
    }
}
static void Tick(bool main_loop)
{
    FakeRobot_AdvanceUs(1000);App_ISR_OnMillisecond();
    if(main_loop)App_RunOnce();
}
static void ModelInputs(void)
{
    SvcHomingSequenceStatus s=Status();
    for(uint8_t a=0;a<3;a++){
        bool low=false;
        if(a==s.flow.axis){
            low=(s.flow.phase==HOME_SEEK && s.flow.seek[a]>=3U) || s.flow.phase==HOME_CONTACT ||
                (s.flow.phase==HOME_BACKOFF && (s.flow.back[a]<2U || !strcmp(mode,"stuck")));
            if(!strcmp(mode,"stuck") && s.flow.phase==HOME_RELEASE)low=true;
        }
        if(a>s.flow.axis && !strcmp(mode,"boot_pressed") && a==1)low=true;
        bool falling=!fake_limits[a] && low;fake_limits[a]=low;if(falling)Edge(a);
    }
}
static void Drive(void)
{
    for(unsigned t=0;t<220000 && !SafetyService_IsMotionAllowed();t++){
        SvcHomingSequenceStatus before=Status();
        if(before.flow.phase==HOME_FAILED)break;
        ModelInputs();Tick(true);
        assert(FakeRobot_CriticalDepth()==0);
        SvcHomingSequenceStatus s=Status();
        /* 验证真实Service采用独立退回周期，而非只在纯算法测试中设置。 */
        assert(s.flow.config.period_ms==HOMING_SEQUENCE_SEEK_PERIOD_MS &&
               s.flow.config.backoff_period_ms==HOMING_SEQUENCE_BACKOFF_PERIOD_MS);
        if(!s.committed)assert(!SafetyService_IsMotionAllowed());
        if(!strcmp(mode,"calibration_fail") && s.flow.phase==HOME_RELEASE && s.flow.axis==2)
            fake_encoder_failure[0]=true;
        if(!strcmp(mode,"retrigger") && s.flow.rearmed && s.flow.phase==HOME_BACKOFF){
            fake_limits[0]=true;Edge(0);Tick(true);
#if ROBOTARM_LIMIT_STATE_MODE
            assert(Status().flow.holding && !fake_joints_enabled && !SafetyService_HasFault());
            for(unsigned i=0;i<10;i++)Tick(true); /* 首样本已停步，持续10ms再锁定。 */
#endif
            break;
        }
    }
    App_RunOnce();
}
/** 从真实回零成功状态继续协议生命周期；替身不模拟电机负载和重力。 */
#define LCHECK(c) do {if(!(c)){fprintf(stderr,"lifecycle line %d: %s\n",__LINE__,#c);exit(2);}}while(0)
static void Lifecycle(void)
{
    LCHECK(SafetyService_IsMotionAllowed() && fake_joints_enabled);
    Send("M408");LCHECK(strstr(fake_tx,"STATE:READY HOMED:1 ALLOWED:1"));
    Send("M409");LCHECK(strstr(fake_tx,"FRAME:JOINT"));
    if(!strcmp(mode,"lifecycle_move")){
        /* 到位后给闭环提供模拟编码器反馈，再验证 M400。 */
        uint32_t before=fake_steps[0]+fake_steps[1]+fake_steps[2];
        Send("G1 X1 Y185 Z240 F300");
        LCHECK(!strstr(fake_tx,"error:"));
        TIM_HandleTypeDef timer={.Instance=TIM6};
        unsigned t;
        for(t=0;t<300000;t++){
            FakeRobot_AdvanceUs(20U);App_ISR_OnTimerElapsed(&timer);
            if(t%50U==0U){App_ISR_OnMillisecond();App_RunOnce();}
            if(!CommandService_IsMotionPending() && MotionService_IsIdle())break;
        }
        LCHECK(t<300000 && !SafetyService_HasFault());
        LCHECK(fake_steps[0]+fake_steps[1]+fake_steps[2]>before);
        ClosedLoopStatus_t status;
        MotionService_GetClosedLoopStatus(&status);
        memcpy(fake_motor_deg, status.target_motor_deg, sizeof(fake_motor_deg));
        Send("M400");
        LCHECK(!strstr(fake_tx,"ok\r\n"));
        for (unsigned ms=0;ms<CL_SETTLE_MS+100U && !strstr(fake_tx,"ok\r\n");ms++) Tick(true);
        LCHECK(strstr(fake_tx,"ok\r\n"));
        LCHECK(fake_joints_enabled);return;
    }
    if(!strcmp(mode,"lifecycle_gripper")){
        unsigned opened=fake_gripper_open_count,closed=fake_gripper_close_count;
        Send("M3");LCHECK(strstr(fake_tx,"M3OK") && fake_gripper_open_count==opened+1U);
        for(unsigned t=0;t<601;t++)Tick(true);
        Send("M5");LCHECK(strstr(fake_tx,"M5OK") && fake_gripper_close_count==closed+1U);
        LCHECK(SafetyService_IsMotionAllowed());return;
    }
    /* 停止必须清规划、路径队列和STEP队列，不能只停当前帧。 */
    Send("G1 X1 Y185 Z240 F300");
    Send("M400");Send("M112");
    LCHECK(strstr(fake_tx,"M112OK") && SafetyService_GetFault()==SAFETY_FAULT_HOST_STOP);
    LCHECK(!SafetyService_IsMotionAllowed() && !fake_joints_enabled);
    LCHECK(MotionService_IsIdle() && !CommandService_IsMotionPending() && !CommandService_GetQueuedMoveCount());
    uint32_t steps[3];for(unsigned a=0;a<3;a++)steps[a]=fake_steps[a];
    unsigned opened=fake_gripper_open_count;
    Send("M3");LCHECK(strstr(fake_tx,"error:") && fake_gripper_open_count==opened);
    Send("M114");LCHECK(strstr(fake_tx,"error:"));
    for(unsigned t=0;t<100;t++)Tick(true);
    for(unsigned a=0;a<3;a++)LCHECK(fake_steps[a]==steps[a]);
    if(!strcmp(mode,"lifecycle_stop"))return;
    if(!strcmp(mode,"lifecycle_driver_retry")){
        fake_driver_prepare_ok=false;Send("M999");
        LCHECK(SafetyService_HasFault() && !fake_joints_enabled && !SafetyService_IsMotionAllowed());
        unsigned prepares=fake_configure_count;
        for(unsigned t=0;t<100;t++)Tick(true);
        LCHECK(fake_configure_count==prepares); /* 不自动重试驱动配置。 */
        fake_driver_prepare_ok=true;
    }
    Send("M999");LCHECK(strstr(fake_tx,"M999: homing started"));
    LCHECK(SafetyService_HasFault() && !SafetyService_IsMotionAllowed());
    Drive();LCHECK(SafetyService_IsMotionAllowed() && !SafetyService_HasFault() && Status().committed);
    /* M999OK由App原生printf输出到CTest日志，不经过fake_tx；
     * 此处核对真实参考提交，完整回复握手另由host_client_tests验证。 */
    Send("M409");LCHECK(strstr(fake_tx,"FRAME:JOINT"));
    Send("M3");LCHECK(strstr(fake_tx,"M3OK") && fake_gripper_open_count==opened+1U);
}
#undef LCHECK

int main(int argc,char **argv)
{
    assert(argc==2);mode=argv[1];FakeRobot_Reset();FakeHardware_Reset();App_Teleop_SetMode(SYS_MODE_GCODE);
#if ROBOTARM_LIMIT_STATE_MODE
    if(!strcmp(mode,"state_noise"))for(unsigned i=0;i<4096;i++)for(uint8_t a=0;a<3;a++)Edge(a);
    if(!strcmp(mode,"state_latched"))for(unsigned i=0;i<256;i++)(void)LimitIrqGuard_OnEdgeFromISR(1);
#endif
    if(!strcmp(mode,"early_irq"))for(unsigned i=0;i<256;i++)Edge(1);
    App_ISR_OnLimitsConfigured();
    if(!strcmp(mode,"boot_pressed")){
        fake_limits[1]=true;Edge(1);
        for(unsigned i=0;i<4096;i++)Edge(1);
        for(unsigned i=0;i<11;i++)Tick(false);
        assert(masks[1]==1 && !LimitIrqGuard_GetLatchMask());
    }
    if(!strcmp(mode,"gripper_fail"))fake_gripper_init_failure=true;
    if(!strcmp(mode,"rearm_fail") || !strcmp(mode,"rearm_recovery"))refuse_rearm=true;
    App_Init();App_RunOnce();
    if(!strcmp(mode,"early_irq") || !strcmp(mode,"state_latched")){
        assert(LimitIrqGuard_GetLatchMask()==2 && !fake_joints_enabled);
        unsigned n=fake_enable_true_count;Send("M999");
        assert(fake_enable_true_count==n && SafetyService_GetFault()==SAFETY_FAULT_LIMIT_IRQ_OVERLOAD);
    }else if(!strcmp(mode,"gripper_fail")){
        assert(!fake_enable_true_count && !fake_gripper_init_count && SafetyService_HasFault());
    }else if(!strcmp(mode,"irq")){
        for(unsigned i=0;i<256;i++)Edge(2);
        Tick(true);assert(!fake_joints_enabled && SafetyService_GetFault()==SAFETY_FAULT_LIMIT_IRQ_OVERLOAD);
        Send("M999");assert(!SafetyService_IsMotionAllowed() && LimitIrqGuard_GetLatchMask()==4);
    }else if(!strcmp(mode,"stall")){
        for(unsigned i=0;i<251;i++)Tick(false);
        assert(Status().flow.reason==HOME_MAIN_STALLED && !fake_joints_enabled);
        assert(fake_wave_rises[0]<=250U/Status().flow.config.period_ms && !fake_wave_level[0]);
    }else if(!strcmp(mode,"contact_flood")){
        fake_limits[0]=true;Edge(0);for(unsigned i=0;i<4096;i++)Edge(0);
        assert(!fake_joints_enabled && Status().flow.phase==HOME_CONTACT && masks[0]==1);
        assert(!LimitIrqGuard_GetLatchMask());Drive();
        assert(SafetyService_IsMotionAllowed());
    }else{
#if ROBOTARM_LIMIT_STATE_MODE
        if(!strcmp(mode,"seek_no30s") || !strcmp(mode,"seek_total_deadline")){
            unsigned duration=!strcmp(mode,"seek_no30s")?31000U:220010U;
            for(unsigned t=0;t<duration && Status().flow.phase!=HOME_FAILED;t++)Tick(true);
            if(!strcmp(mode,"seek_no30s")){
                assert(Status().flow.phase==HOME_SEEK && Status().flow.reason==HOME_OK);
                assert(Status().flow.seek[0]>15000U && Status().flow.config.seek_timeout_ms==0U);
            }else assert(Status().flow.reason==HOME_TOTAL_TIMEOUT && !fake_joints_enabled && SafetyService_HasFault());
            puts("seek deadline integration passed");return 0;
        }
        if(!strcmp(mode,"state_noise"))assert(!LimitIrqGuard_GetLatchMask());
        if(!strcmp(mode,"contact_glitch") || !strcmp(mode,"contact_repeat")){
            unsigned count=!strcmp(mode,"contact_repeat")?3U:1U;
            for(unsigned n=0;n<count;n++){
                unsigned before=fake_wave_rises[0];fake_limits[0]=true;Tick(true);
                assert(Status().flow.phase==HOME_CONTACT && !fake_joints_enabled && fake_wave_rises[0]==before);
                fake_limits[0]=false;
                for(unsigned t=0;t<10;t++){
                    Tick(true);assert(Status().flow.phase==HOME_CONTACT && !fake_joints_enabled);
                    assert(fake_wave_rises[0]==before);
                }
                Tick(true);assert(Status().flow.phase==HOME_SEEK && !SafetyService_HasFault());
                assert(fake_wave_rises[0]==before && Status().flow.contact_cancel_count==n+1U);
            }
            Send("M127");assert(strstr(fake_tx,"CONTACT_CANCELS:") && strstr(fake_tx,"BACK_PERIOD_MS:1"));
        }
        if(!strncmp(mode,"other_",6)){
            assert(Status().flow.config.qualify_unexpected);
            if(!strcmp(mode,"other_backoff")){
                fake_limits[0]=true;
                for(unsigned t=0;t<11;t++)Tick(true);
                assert(Status().flow.phase==HOME_BACKOFF);fake_limits[0]=false;
            }
            unsigned before=fake_wave_rises[0];fake_limits[1]=true;Tick(true);
            assert(Status().flow.holding && !fake_joints_enabled && fake_wave_rises[0]==before);
            assert(!SafetyService_HasFault());
            if(!strcmp(mode,"other_held") || !strcmp(mode,"other_unstable")){
                for(unsigned t=0;t<101 && Status().flow.phase!=HOME_FAILED;t++){
                    if(!strcmp(mode,"other_unstable"))fake_limits[1]=(t&1U)!=0;
                    Tick(true);assert(fake_wave_rises[0]==before);
                }
                assert(Status().flow.reason==(!strcmp(mode,"other_held")?HOME_UNEXPECTED:HOME_UNSTABLE));
                assert(!fake_joints_enabled && SafetyService_HasFault());
                if(!strcmp(mode,"other_held"))assert(Status().flow.failure_low&2U);
                Send("M127");assert(strstr(fake_tx,"M127 CONFIRM_OTHER:1") && strstr(fake_tx,"BACK_PERIOD_MS:1"));
                puts("other input failure integration passed");return 0;
            }
            fake_limits[1]=false;for(unsigned t=0;t<10;t++){
                Tick(true);assert(fake_wave_rises[0]==before && !fake_joints_enabled);
            }
            Tick(true);assert(!Status().flow.holding && !SafetyService_HasFault());
            Send("M127");assert(strstr(fake_tx,"M127 CONFIRM_OTHER:1"));
        }
        if(!strcmp(mode,"state_stop") || !strcmp(mode,"state_bounce") || !strcmp(mode,"state_unstable")){
            unsigned steps=fake_wave_rises[0];fake_limits[0]=true;Tick(false);
            assert(Status().flow.phase==HOME_CONTACT && !fake_joints_enabled && !fake_wave_level[0]);
            assert(fake_wave_rises[0]==steps); /* 首次样本停步，先于稳定确认。 */
            if(!strcmp(mode,"state_unstable")){
                for(unsigned t=0;t<101;t++){fake_limits[0]=(t&1U)!=0;Tick(true);}
                assert(Status().flow.reason==HOME_UNSTABLE && !fake_joints_enabled);
                assert(fake_wave_rises[0]==steps && SafetyService_HasFault());
                puts("state_unstable passed");return 0;
            }
            if(!strcmp(mode,"state_bounce"))for(unsigned t=0;t<8;t++){fake_limits[0]=(t&1U)!=0;Tick(false);}
            fake_limits[0]=true;
            /* 观察稳定确认的状态切换本拍，不把下一个1ms退回STEP误算为接触脉冲。
             * 旧2ms周期恰好掩盖了固定多采一拍的测试假设。 */
            for(unsigned t=0;t<20 && Status().flow.phase==HOME_CONTACT;t++)Tick(false);
            assert(Status().flow.phase==HOME_BACKOFF && Status().flow.contacts==1);
            assert(fake_wave_rises[0]==steps); /* 尚未到反向STEP周期。 */
        }
#endif
        Drive();
        if(!strcmp(mode,"stuck") || !strcmp(mode,"rearm_fail") || !strcmp(mode,"rearm_recovery") || !strcmp(mode,"calibration_fail") || !strcmp(mode,"retrigger") ||
           !strcmp(mode,"calibration_irq") || !strcmp(mode,"handoff_irq") || !strcmp(mode,"handoff_limit")){
            assert(!SafetyService_IsMotionAllowed() && !fake_joints_enabled);
            assert(SafetyService_HasFault());
            if(!strcmp(mode,"rearm_fail"))assert(Status().flow.reason==HOME_PORT);
            if(!strcmp(mode,"retrigger"))assert(Status().flow.reason==HOME_UNEXPECTED);
            if(!strcmp(mode,"handoff_limit"))assert(Status().flow.reason==HOME_UNEXPECTED);
            if(!strcmp(mode,"handoff_irq") || !strcmp(mode,"calibration_irq"))
                assert(SafetyService_GetFault()==SAFETY_FAULT_LIMIT_IRQ_OVERLOAD && LimitIrqGuard_GetLatchMask()==2);
            for(unsigned i=0;i<300;i++)Tick(true);
            assert(!fake_joints_enabled && !SafetyService_IsMotionAllowed());
            if(!strcmp(mode,"rearm_recovery")){
                refuse_rearm=false;fake_limits[0]=fake_limits[1]=fake_limits[2]=false;
                Send("M999");assert(SafetyService_HasFault() && !SafetyService_IsMotionAllowed());
                for(unsigned i=0;i<10;i++)Tick(true);
                Drive();assert(SafetyService_IsMotionAllowed() && !SafetyService_HasFault());
            }
        }else{
            SvcHomingSequenceStatus s=Status();
            assert(s.committed && s.flow.phase==HOME_DONE && s.flow.completed==7 && s.flow.rearmed==7);
            assert(!fake_limit_irq_mask && SafetyService_IsMotionAllowed() && !SafetyService_HasFault());
            assert(s.flow.back[0]==4000 && s.flow.back[1]==1240 && s.flow.back[2]==400);
            for(unsigned a=0;a<3;a++)assert(rearms[a]==1);
            Send("M126");assert(strstr(fake_tx,"COMPLETE:7 OWNED:0 REARMED:7") && strstr(fake_tx,"COMMITTED:1"));
            Send("M115");assert(strstr(fake_tx,"UART:115200") && strstr(fake_tx,"SEQUENTIAL_HOME"));
            if(!strcmp(mode,"closed_loop_boot") || !strcmp(mode,"closed_loop_recovery")){
                /* 真实App在回零完成时只Init闭环，不能先调用Sync来掩盖诊断遗漏。
                 * 首条G1之前的默认零目标已经存在，M411应与Safety READY参考一致。 */
                if(!strcmp(mode,"closed_loop_boot"))CheckClosedLoopReference("REF:1 CONTROL:1 ACTIVE:0");
                if(!strcmp(mode,"closed_loop_recovery")){
                    Send("M112");assert(strstr(fake_tx,"M112OK"));
                    CheckClosedLoopReference("REF:0 CONTROL:0 ACTIVE:0");
                    Send("M999");Drive();assert(SafetyService_IsMotionAllowed());
                    CheckClosedLoopReference("REF:1 CONTROL:1 ACTIVE:0");
                }
                puts("closed-loop initial reference lifecycle passed");return 0;
            }
            if(!strncmp(mode,"lifecycle_",10)){
                Lifecycle();puts("full GPIO software lifecycle passed");return 0;
            }
#if ROBOTARM_LIMIT_STATE_MODE
            assert(strstr(fake_tx,"LIMIT_STATE_1MS"));
            if(!strncmp(mode,"runtime_",8)){
                unsigned before=fake_wave_rises[1];fake_limits[1]=true;
                if(!strcmp(mode,"runtime_stop_fail"))fake_step_failure[1]=true;
                TIM_HandleTypeDef timer={.Instance=TIM6};App_ISR_OnTimerElapsed(&timer);
                assert(!fake_wave_level[1] && fake_wave_rises[1]==before);
                if(!strcmp(mode,"runtime_stop_fail")){
                    assert(!fake_joints_enabled && SafetyService_GetFault()==SAFETY_FAULT_INTERNAL);
                    assert(!SafetyService_IsMotionAllowed());puts("runtime stop port failure passed");return 0;
                }
                assert(!SafetyService_HasFault() && fake_joints_enabled);
                if(!strcmp(mode,"runtime_glitch")){
                    fake_limits[1]=false;
                    for(unsigned t=0;t<11;t++)Tick(false);
                    assert(!SafetyService_HasFault() && SafetyService_IsMotionAllowed() && fake_joints_enabled);
                }else{
                    if(!strcmp(mode,"runtime_unstable")){
                        for(unsigned t=0;t<101;t++){fake_limits[1]=(t&1U)!=0;Tick(false);}
                        assert(SafetyService_GetFault()==SAFETY_FAULT_INPUT_LOST);
                    }else{
                        for(unsigned t=0;t<9;t++)Tick(false);
                        assert(!SafetyService_HasFault());Tick(false);
                        assert(SafetyService_GetFault()==SAFETY_FAULT_LIMIT_SWITCH);
                    }
                    assert(!fake_joints_enabled && !SafetyService_IsMotionAllowed());
                    assert(fake_wave_rises[1]==before);
                    if(!strcmp(mode,"runtime_recovery")){
                        fake_limits[0]=fake_limits[1]=fake_limits[2]=false;
                        Send("M999");assert(SafetyService_HasFault());Drive();
                        assert(SafetyService_IsMotionAllowed() && !SafetyService_HasFault());
                    }else{
                        SafetyFault_t first=SafetyService_GetFault();SafetyService_ReportHostStop();
                        assert(SafetyService_GetFault()==first);
                    }
                }
                puts("runtime input scenario passed");return 0;
            }
#endif
            if(!strcmp(mode,"recovery")){
                Send("M112");assert(SafetyService_GetFault()==SAFETY_FAULT_HOST_STOP);
                Send("M999");assert(!SafetyService_IsMotionAllowed() && SafetyService_HasFault());
                Drive();assert(SafetyService_IsMotionAllowed() && !SafetyService_HasFault());
                assert(Status().committed);
            }
            if(!strcmp(mode,"ready_hold")){
                unsigned before[3];for(unsigned a=0;a<3;a++)before[a]=fake_wave_rises[a];
                TIM_HandleTypeDef timer={.Instance=TIM6};
                for(unsigned t=0;t<200;t++){
                    assert(AppHardware_StopJointSteps()); /* 停STEP不释放保持力。 */
                    Tick(true);
                    App_ISR_OnTimerElapsed(&timer); /* 空闲发步定时入口也不补发动作。 */
                    assert(fake_joints_enabled && SafetyService_IsMotionAllowed());
                }
                for(unsigned a=0;a<3;a++)assert(fake_wave_rises[a]==before[a]);
                Send("M409");assert(strstr(fake_tx,"FRAME:JOINT"));
            }
            if(!strcmp(mode,"release_reference")){
                AppHardware_StopActuators();SafetyService_RevokePermission();
                assert(!fake_joints_enabled && !SafetyService_IsMotionAllowed() && !SafetyService_HasFault());
                Send("M114");assert(strstr(fake_tx,"error:") && !strstr(fake_tx,"M114 PLAN"));
                Send("M409");assert(strstr(fake_tx,"FRAME:UNAVAILABLE"));
                unsigned before[3];for(unsigned a=0;a<3;a++)before[a]=fake_wave_rises[a];
                Send("G1 X1 F100");assert(strstr(fake_tx,"error:"));
                for(unsigned a=0;a<3;a++)assert(fake_wave_rises[a]==before[a]);
                assert(!fake_joints_enabled && !SafetyService_IsMotionAllowed());
            }
            SafetyService_RevokePermission();assert(!SafetyService_IsMotionAllowed() && !SafetyService_HasFault());
            SafetyService_ReportHostStop();SafetyService_RevokePermission();
            assert(SafetyService_GetFault()==SAFETY_FAULT_HOST_STOP);
        }
    }
    printf("sequence_runtime %s passed\n",mode);return 0;
}
