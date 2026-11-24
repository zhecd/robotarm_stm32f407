/** @file guard.c
 * @brief 真实正常App的单次低速启动试验外壳；不复制回零或标定算法。
 * @details 等待START才推进App；停止后仅查询，不运行READY周期任务。
 * 完成时先记录正常App实际HOMED/ALLOWED，再主动撤销许可，禁止后续联动。
 */
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "app_init.h"
#include "app_hardware_adapter.h"
#include "app/app_serial_protocol.h"
#include "service/svc_homing.h"
#include "safety_service.h"
#include "driver/drv_servo.h"
#include "device/dev_joint.h"
#include "limit_irq_guard.h"
#include "trial_logic.h"
#include "ready_hold.h"
#include "motion_service.h"
#include "command_service.h"
#ifndef STARTUP_TRIAL_READY_HOLD_MS
#define STARTUP_TRIAL_READY_HOLD_MS 0
#endif
#define TRIAL_STR_INNER(x) #x
#define TRIAL_STR(x) TRIAL_STR_INNER(x)
#if STARTUP_TRIAL_READY_HOLD_MS > 0
#define READY_HOLD_FIELD " READY_HOLD_MS:" TRIAL_STR(STARTUP_TRIAL_READY_HOLD_MS)
#define READY_HOLD_FEATURE ",READY_HOLD,M128"
#else
#define READY_HOLD_FIELD ""
#define READY_HOLD_FEATURE ""
#endif
#if ROBOTARM_PARALLEL_HOMING
#define HOME_ORDER_FIELD " SEQUENCE:PARALLEL_BARRIER"
#define HOME_ORDER_FEATURE ",PARALLEL_HOME,PS2_DIAGNOSTICS"
#else
#define HOME_ORDER_FIELD " SEQUENCE:0,1,2"
#define HOME_ORDER_FEATURE ""
#endif
#if ROBOTARM_SEQUENTIAL_HOMING
#include "service/svc_homing_sequence.h"
#endif
#include <stdio.h>
#include <string.h>
#include <math.h>
#ifndef STARTUP_TRIAL_MOTION_ENABLED
#define STARTUP_TRIAL_MOTION_ENABLED 0
#endif
#if ROBOTARM_LIMIT_STATE_MODE
#define SEQUENCE_INPUT_FEATURE ",LIMIT_STATE_1MS,HOME_INPUT_CONFIRM,M127,CONTACT_CANCEL_10MS"
#else
#define SEQUENCE_INPUT_FEATURE ""
#endif
/* STARTUP8声明M3退回400微步；主机必须核对本轮真实参数。 */
#if STARTUP_TRIAL_READY_HOLD_MS > 0
#define SEQUENCE_TRIAL_PROTOCOL "STARTUP9"
#elif ROBOTARM_LIMIT_STATE_MODE
#define SEQUENCE_TRIAL_PROTOCOL "STARTUP8"
#else
#define SEQUENCE_TRIAL_PROTOCOL "STARTUP8"
#endif
static volatile Trial trial;
static volatile uint32_t main_seen;
static AppSerialMode_t app_mode=APP_SERIAL_CHECKING;
static volatile bool cleanup;
static volatile bool initialized;
static bool watchdog_started;
static ReadyHold ready_hold;
static volatile bool ready_waiting;
bool AppTrial_ReleasePreparedStartup(void);
static volatile uint32_t pwm_started;
static uint32_t feedback_ms,feedback_seq;
static uint8_t feedback_mask;
static uint8_t feedback_epoch;
static float feedback[3];
/* SWD只读取证；READY边界快照不等于试验结束后仍具有动作许可。 */
volatile struct {
    uint32_t magic,seek[3],back[3],ready_homed,ready_allowed,timer_suppressed;
} g_startup_trial={.magic=0x54524931U};
static Trial Snapshot(void)
{uint32_t k=__get_PRIMASK();__disable_irq();Trial t=trial;__set_PRIMASK(k);return t;}
static void CutPWM(void)
{
    if(RCC->APB1ENR&RCC_APB1ENR_TIM2EN){TIM2->CCER=0;TIM2->CR1=0;}
    GPIOA->BSRR=GPIO_PIN_1<<16;
    GPIOA->MODER=(GPIOA->MODER&~(3U<<2))|(1U<<2);
}
static void Cut(void)
{
    GPIOD->BSRR=GPIO_PIN_2;
    GPIOA->BSRR=(GPIO_PIN_6|GPIO_PIN_7)<<16;GPIOB->BSRR=GPIO_PIN_0<<16;
    CutPWM();
    if(RCC->APB1ENR&RCC_APB1ENR_TIM6EN){TIM6->CR1=0;TIM6->DIER=0;}
    if(RCC->APB1ENR&RCC_APB1ENR_TIM5EN){TIM5->CR1=0;TIM5->DIER=0;}
    __DSB();
}
static void Stop(TrialReason why)
{
    uint32_t k=__get_PRIMASK();__disable_irq();Cut();
    Trial t=trial;Trial_Stop(&t,why);trial=t;cleanup=true;__set_PRIMASK(k);
}
static void EarlyGPIO(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_GPIOD_CLK_ENABLE();Cut();
    GPIO_InitTypeDef g={.Mode=GPIO_MODE_OUTPUT_PP,.Pull=GPIO_NOPULL,.Speed=GPIO_SPEED_FREQ_LOW};
    g.Pin=GPIO_PIN_2;HAL_GPIO_Init(GPIOD,&g);
    g.Pin=GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7;HAL_GPIO_Init(GPIOA,&g);
    g.Pin=GPIO_PIN_0;HAL_GPIO_Init(GPIOB,&g);
}
void __real_HAL_IncTick(void);
void __wrap_HAL_IncTick(void)
{
    __real_HAL_IncTick();uint32_t k=__get_PRIMASK();__disable_irq();
    Trial t=trial;bool was=t.state==TRIAL_ACTIVE;
    Trial_Tick(&t,HAL_GetTick(),main_seen);trial=t;
    if(t.state!=TRIAL_ACTIVE){Cut();if(was)cleanup=true;}
    else if((RCC->APB1ENR&RCC_APB1ENR_TIM2EN) && (TIM2->CCER&TIM_CCER_CC2E) &&
            (uint32_t)(HAL_GetTick()-pwm_started)>=700U)Stop(TRIAL_PWM_OVERRUN);
    /* 指示灯只映射实时输入，不将可见LED当作编码器/机械位置测量。 */
    if(initialized){
        uint32_t m=(~GPIOC->IDR>>10)&7U;
        GPIOC->BSRR=(((m<<1)|1U)<<16)|((~(m<<1))&14U);
    }
    __set_PRIMASK(k);
}
void TIM5_IRQHandler(void)
{if((TIM5->SR&TIM_SR_UIF)&&(TIM5->DIER&TIM_DIER_UIE)){TIM5->SR=0;Stop(TRIAL_DEADLINE);}}
static void ArmDeadline(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();TIM5->CR1=0;TIM5->DIER=0;
    /* APB1定时器84MHz，10kHz计数；32位TIM5允许900000个计数。 */
    TIM5->PSC=8399U;TIM5->ARR=TRIAL_DEADLINE_MS*10U-1U;TIM5->CNT=0;
    TIM5->EGR=TIM_EGR_UG;TIM5->SR=0;HAL_NVIC_ClearPendingIRQ(TIM5_IRQn);
    HAL_NVIC_SetPriority(TIM5_IRQn,0,0);HAL_NVIC_EnableIRQ(TIM5_IRQn);
    TIM5->DIER=TIM_DIER_UIE;TIM5->CR1=TIM_CR1_OPM|TIM_CR1_CEN;
    DBGMCU->APB1FZ&=~(DBGMCU_APB1_FZ_DBG_TIM5_STOP|DBGMCU_APB1_FZ_DBG_IWDG_STOP);
}
static bool InitWatchdog(void)
{
    RCC->CSR|=RCC_CSR_LSION;uint32_t began=HAL_GetTick();
    while(!(RCC->CSR&RCC_CSR_LSIRDY))if((uint32_t)(HAL_GetTick()-began)>100U)return false;
    IWDG->KR=0xcccc;IWDG->KR=0x5555;IWDG->PR=3;IWDG->RLR=499;
    began=HAL_GetTick();while(IWDG->SR)if((uint32_t)(HAL_GetTick()-began)>100U)return false;
    IWDG->KR=0xaaaa;watchdog_started=true;return true;
}
void __real_HAL_GPIO_WritePin(GPIO_TypeDef *,uint16_t,GPIO_PinState);
void __wrap_HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,GPIO_PinState value)
{
    uint32_t k=__get_PRIMASK();__disable_irq();
    bool actuator=(p==GPIOD&&(pin&GPIO_PIN_2)&&value==GPIO_PIN_RESET)||
        (p==GPIOA&&(pin&(GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7))&&value==GPIO_PIN_SET)||
        (p==GPIOB&&(pin&GPIO_PIN_0)&&value==GPIO_PIN_SET);
    /* READY验收窗口只验证保持力，任何新的STEP高电平请求都终止试验。 */
    bool step_high=value==GPIO_PIN_SET && ((p==GPIOA&&(pin&(GPIO_PIN_6|GPIO_PIN_7))) || (p==GPIOB&&(pin&GPIO_PIN_0)));
    if(ready_waiting && step_high){Stop(TRIAL_OUTPUT);__set_PRIMASK(k);return;}
    if(actuator && trial.state!=TRIAL_ACTIVE){Cut();__set_PRIMASK(k);return;}
    __real_HAL_GPIO_WritePin(p,pin,value);
    __set_PRIMASK(k);
}
HAL_StatusTypeDef __real_HAL_TIM_PWM_Start(TIM_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef __wrap_HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t channel)
{
#if STARTUP_TRIAL_MOTION_ENABLED
    uint32_t k=__get_PRIMASK();__disable_irq();
    if(ready_waiting){Stop(TRIAL_OUTPUT);__set_PRIMASK(k);return HAL_ERROR;}
    if(trial.state==TRIAL_ACTIVE && t->Instance==TIM2 && channel==TIM_CHANNEL_2){
        pwm_started=HAL_GetTick();
        GPIOA->MODER=(GPIOA->MODER&~(3U<<2))|(2U<<2);
        HAL_StatusTypeDef result=__real_HAL_TIM_PWM_Start(t,channel);__set_PRIMASK(k);return result;
    }
    __set_PRIMASK(k);
#else
    (void)t;(void)channel;
#endif
    Stop(TRIAL_OUTPUT);return HAL_ERROR;
}
/* 正常App仍走标定/READY提交，但本试验禁止启动后续闭环STEP定时器。 */
void __wrap_AppHardware_StartStepTimer(void){++g_startup_trial.timer_suppressed;}
bool __real_Dev_Joint_Step(DevJointId_t);
bool __wrap_Dev_Joint_Step(DevJointId_t axis)
{
#if STARTUP_TRIAL_MOTION_ENABLED && !ROBOTARM_SEQUENTIAL_HOMING
    static const uint32_t cap[3]={4000U,1240U,200U};
    uint32_t k=__get_PRIMASK();__disable_irq();
    if((unsigned)axis<3U && trial.state==TRIAL_ACTIVE){
        SvcHomingState_t phase=Svc_Homing_GetState();
        volatile uint32_t *count=phase==SVC_HOMING_SEEK?g_startup_trial.seek:g_startup_trial.back;
        uint32_t limit=phase==SVC_HOMING_SEEK?200U:cap[axis];
        if((phase==SVC_HOMING_SEEK || phase==SVC_HOMING_BACKOFF) && count[axis]<limit){
            if(__real_Dev_Joint_Step(axis)){++count[axis];__set_PRIMASK(k);return true;}
        }
    }
    __set_PRIMASK(k);
#else
    (void)axis;
#endif
    Stop(TRIAL_OUTPUT);return false;
}
void __real_AppSerial_SetMode(AppSerialMode_t);
void __wrap_AppSerial_SetMode(AppSerialMode_t mode)
{
    app_mode=mode;
    if(mode==APP_SERIAL_READY){
#if ROBOTARM_SEQUENTIAL_HOMING
        SvcHomingSequenceStatus h;Svc_HomingSequence_GetStatus(&h);
        for(unsigned a=0;a<3;a++){g_startup_trial.seek[a]=h.flow.seek[a];g_startup_trial.back[a]=h.flow.back[a];}
#endif
        SafetyServiceStatus_t s;SafetyService_GetStatus(&s);
        g_startup_trial.ready_homed=s.homed;g_startup_trial.ready_allowed=s.motion_allowed;
#if STARTUP_TRIAL_READY_HOLD_MS > 0
        if(s.homed && s.motion_allowed && !s.fault_latched &&
           ReadyHold_Arm(&ready_hold,HAL_GetTick(),STARTUP_TRIAL_READY_HOLD_MS))ready_waiting=true;
        else Stop(TRIAL_APP_FAILED);
#else
        Stop(s.homed && s.motion_allowed?TRIAL_COMPLETE:TRIAL_APP_FAILED);
#endif
        mode=APP_SERIAL_OUTPUT_LOCKED;
    }else if(mode==APP_SERIAL_FAILED)Stop(TRIAL_APP_FAILED);
    __real_AppSerial_SetMode(mode);
}
static void SampleFeedback(void)
{
    uint32_t now=HAL_GetTick();if(feedback_seq && (uint32_t)(now-feedback_ms)<100U)return;
    uint8_t mask=0;
    for(uint8_t a=0;a<3;a++)if(AppHardware_ReadMotorAngle(a,&feedback[a]) && isfinite(feedback[a]))mask|=1U<<a;
    feedback_mask=mask;feedback_ms=now;++feedback_seq;
    feedback_epoch=g_startup_trial.ready_homed?1U:0U;
    if(Snapshot().state==TRIAL_ACTIVE && (mask!=7U || (uint32_t)(HAL_GetTick()-now)>=150U))Stop(TRIAL_ENCODER);
}
bool __real_AppHardware_ReadLine(char *,size_t);
bool __wrap_AppHardware_ReadLine(char *out,size_t size)
{
    if(!__real_AppHardware_ReadLine(out,size))return false;
    if(!strcmp(out,"M128")){
        printf("M128 HOLD_STATE:%u DURATION_MS:%lu ELAPSED_MS:%lu\r\n",ready_hold.state,
            (unsigned long)ready_hold.duration_ms,(unsigned long)(ready_hold.state?HAL_GetTick()-ready_hold.began_ms:0));
        return false;
    }
#if ROBOTARM_SEQUENTIAL_HOMING
    if(!strcmp(out,"M126") || !strcmp(out,"M127"))return true;
#endif
    if(Trial_IsQuery(out))return true;
    if(!strcmp(out,"M112")){Stop(TRIAL_HOST_STOP);return true;}
    if(!strcmp(out,"M115")){
#if ROBOTARM_SEQUENTIAL_HOMING
        printf("M115 FIRMWARE:RobotArmStartupTrial PROTOCOL:" SEQUENCE_TRIAL_PROTOCOL " UART:115200 MOTION_COMPILED:%u AUTO_MOTION:0 SEEK_TIMEOUT_MS:0 STEP_US:%lu BACKOFF_STEP_US:%lu DEADLINE_MS:220000 BACKOFF:4000,1240,400" HOME_ORDER_FIELD " DIR:0,0,1" READY_HOLD_FIELD " FEATURES:IRQ_EVIDENCE,SEQUENTIAL_HOME,M126" SEQUENCE_INPUT_FEATURE HOME_ORDER_FEATURE READY_HOLD_FEATURE "\r\n",STARTUP_TRIAL_MOTION_ENABLED,
            (unsigned long)(HOMING_SEQUENCE_SEEK_PERIOD_MS*1000U),
            (unsigned long)(HOMING_SEQUENCE_BACKOFF_PERIOD_MS*1000U));return false;
#else
        printf("M115 FIRMWARE:RobotArmStartupTrial PROTOCOL:STARTUP1 UART:115200 MOTION_COMPILED:%u AUTO_MOTION:0 SEEK_MAX:200 STEP_US:20000 DEADLINE_MS:90000 BACKOFF:4000,1240,200"
#if ROBOTARM_IRQ_EVIDENCE_MODE
            " FEATURES:IRQ_EVIDENCE"
#endif
            "\r\n",STARTUP_TRIAL_MOTION_ENABLED);return false;
#endif
    }
    if(!strcmp(out,"M123")){
#if ROBOTARM_SEQUENTIAL_HOMING
        SvcHomingSequenceStatus h;Svc_HomingSequence_GetStatus(&h);
        for(unsigned a=0;a<3;a++){g_startup_trial.seek[a]=h.flow.seek[a];g_startup_trial.back[a]=h.flow.back[a];}
#endif
        Trial t=Snapshot();
        printf("M123 STATE:%u REASON:%u USED:%u READY_HOMED:%lu READY_ALLOWED:%lu EN:%u SEEK:%lu,%lu,%lu BACK:%lu,%lu,%lu ELAPSED_MS:%lu\r\n",
            t.state,t.reason,t.used,(unsigned long)g_startup_trial.ready_homed,(unsigned long)g_startup_trial.ready_allowed,
            (GPIOD->IDR&GPIO_PIN_2)?0:1,(unsigned long)g_startup_trial.seek[0],(unsigned long)g_startup_trial.seek[1],(unsigned long)g_startup_trial.seek[2],
            (unsigned long)g_startup_trial.back[0],(unsigned long)g_startup_trial.back[1],(unsigned long)g_startup_trial.back[2],
            (unsigned long)(t.used?HAL_GetTick()-t.began_ms:0));return false;
    }
    if(!strcmp(out,"M124")){
        printf("M124 FRAME:MOTOR ZERO_EPOCH:%u SEQ:%lu AGE_MS:%lu VALID:%u MOTOR_DEG:%.3f,%.3f,%.3f LIMIT_MASK:%lu\r\n",
            feedback_epoch,(unsigned long)feedback_seq,(unsigned long)(HAL_GetTick()-feedback_ms),feedback_mask,
            feedback[0],feedback[1],feedback[2],(unsigned long)((~GPIOC->IDR>>10)&7U));return false;
    }
    if(!strcmp(out,"KEEP")){
        uint32_t k=__get_PRIMASK();__disable_irq();Trial t=trial;
        bool ok=Trial_Keep(&t,HAL_GetTick());trial=t;if(t.state!=TRIAL_ACTIVE)Cut();__set_PRIMASK(k);
        if(!ok && t.used)Stop(t.reason);
        return false;
    }
    if(!strcmp(out,"START")){
        bool ready=initialized && app_mode==APP_SERIAL_CHECKING && !SafetyService_HasFault() &&
            !LimitIrqGuard_GetLatchMask() && feedback_seq && feedback_mask==7U &&
            (uint32_t)(HAL_GetTick()-feedback_ms)<150U;
#if ROBOTARM_SEQUENTIAL_HOMING
        ready=ready && Svc_HomingSequence_BootReady();
#endif
        if(ready && STARTUP_TRIAL_MOTION_ENABLED && !Snapshot().used)
            ready=AppTrial_ReleasePreparedStartup();
        uint32_t k=__get_PRIMASK();__disable_irq();Trial t=trial;
        bool ok=Trial_Start(&t,HAL_GetTick(),STARTUP_TRIAL_MOTION_ENABLED,ready);
        trial=t;main_seen=HAL_GetTick();if(ok)ArmDeadline();__set_PRIMASK(k);
        if(ok && !InitWatchdog()){Stop(TRIAL_OUTPUT);ok=false;}
        printf(ok?"START ACCEPTED\r\n":"error: START rejected\r\n");return false;
    }
    Stop(TRIAL_INPUT);puts("error: trial command rejected\r");return false;
}
static void FailForever(void){Stop(TRIAL_OUTPUT);__disable_irq();while(1){}}
/* 向量使用启动汇编中的弱别名，不能假定--wrap能重定向已定义的向量。
 * Core真实异常入口显式调用本钩子；最终ELF需核对向量和分支。 */
void StartupTrial_FailFromException(void){FailForever();}
void SystemClock_Config(void);
int __wrap_main(void)
{
    EarlyGPIO();HAL_Init();SystemClock_Config();MX_GPIO_Init();EarlyGPIO();
    MX_USART6_UART_Init();MX_USART1_UART_Init();MX_I2C1_Init();MX_I2C2_Init();MX_I2C3_Init();
    MX_TIM2_Init();MX_TIM6_Init();EarlyGPIO();
    /* 若原配置变化，不可把新参数冒充为本轮固定合同。 */
    if(HOMING_STEP_DELAY_US!=20000 || HOMING_MAX_STEPS!=200 || UNITS_PER_DEGREE!=40.0f ||
       HOMING_BACKOFF_M1_DEG!=100.0f || HOMING_BACKOFF_M2_DEG!=31.0f || HOMING_BACKOFF_M3_DEG!=5.0f ||
       HOMING_SEEK_M1_DIR_HIGH!=0 || HOMING_SEEK_M2_DIR_HIGH!=0 || HOMING_SEEK_M3_DIR_HIGH!=1)FailForever();
#if ROBOTARM_SEQUENTIAL_HOMING
    if(HOMING_SEQUENCE_BACKOFF_M3_DEG!=10.0f)FailForever();
#endif
    App_Init();initialized=true;uint32_t last_feed=HAL_GetTick();
    while(1){
        main_seen=HAL_GetTick();
        if(watchdog_started && main_seen!=last_feed){IWDG->KR=0xaaaa;last_feed=main_seen;}
        uint32_t k=__get_PRIMASK();__disable_irq();bool clean=cleanup;cleanup=false;__set_PRIMASK(k);
        if(clean){
            if(ready_hold.state==READY_HOLD_RUNNING)
                (void)ReadyHold_Tick(&ready_hold,HAL_GetTick(),false,false,true);
            ready_waiting=false;
#if ROBOTARM_SEQUENTIAL_HOMING
            AppHardware_StopActuators();
            TrialReason r=Snapshot().reason;
            if(r==TRIAL_COMPLETE)SafetyService_RevokePermission();
            else{
                Svc_Homing_Cancel();
                if(LimitIrqGuard_GetLatchMask())SafetyService_ReportLimitIrqOverload();
                else if(r==TRIAL_HOST_STOP)SafetyService_ReportHostStop();
                else if(r==TRIAL_ENCODER)SafetyService_ReportEncoderFailure();
                else if(r==TRIAL_LEASE_LOST)SafetyService_ReportInputLost();
                else SafetyService_ReportInternalFailure();
            }
#else
            AppHardware_StopActuators();Svc_Homing_Cancel();SafetyService_ReportHostStop();
#endif
            __real_AppSerial_SetMode(APP_SERIAL_OUTPUT_LOCKED);
        }
        SampleFeedback();
        if(Snapshot().state==TRIAL_ACTIVE){
            App_RunOnce();
            if(ready_waiting && Snapshot().state==TRIAL_ACTIVE){
                SafetyServiceStatus_t s;SafetyService_GetStatus(&s);
                ReadyHoldState h=ReadyHold_Tick(&ready_hold,HAL_GetTick(),s.homed,s.motion_allowed,
                    s.fault_latched || MotionService_HasFault() || LimitIrqGuard_GetLatchMask()!=0U ||
                    !MotionService_IsIdle() || CommandService_IsMotionPending());
                if(h==READY_HOLD_DONE || h==READY_HOLD_FAILED){
                    ready_waiting=false;Stop(h==READY_HOLD_DONE?TRIAL_COMPLETE:TRIAL_APP_FAILED);
                }
            }
        }
        else{Cut();AppSerial_Service();}
    }
}
