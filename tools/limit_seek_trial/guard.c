/** @file guard.c
 * @brief 单次逐轴限位接触试验；复用真实硬件/ISR/过载保护，不运行正常启动。
 * @details 等待START后依次M1 D0、M2 D0、M3 D1；触发即停，无夹爪、退回或标定。
 * 扩展版取消5°正常结束条件，保留每轴30秒/总95秒故障截止；结束耗尽许可。
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
#include "seek_logic.h"
#include "contact_gate.h"
#include "seek_result.h"
#include "boot_contact.h"
#include "input_hold.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#ifndef STARTUP_TRIAL_MOTION_ENABLED
#define STARTUP_TRIAL_MOTION_ENABLED 0
#endif
static volatile Trial trial;
static volatile uint32_t main_seen;
static AppSerialMode_t app_mode=APP_SERIAL_CHECKING;
static volatile bool cleanup;
static volatile bool initialized;
static bool watchdog_started;
static volatile Seek seek;
static SeekContact contact={.seek=&seek};
static SeekBootContact boot_contact;
static SeekInputHold input_hold;
static volatile bool limits_ready;
static bool drivers_prepared;
static volatile bool armed;

static volatile uint32_t feedback_ms;
static uint32_t feedback_seq;
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
/* 接触消抖只停执行器；TIM5总截止继续计时，不能被接触事件撤销。 */
void SeekContactPort_StopOutputs(void)
{
    GPIOD->BSRR=GPIO_PIN_2;
    GPIOA->BSRR=(GPIO_PIN_6|GPIO_PIN_7)<<16;GPIOB->BSRR=GPIO_PIN_0<<16;
    CutPWM();
    if(RCC->APB1ENR&RCC_APB1ENR_TIM6EN){TIM6->CR1=0;TIM6->DIER=0;}
    __DSB();
}
void SeekContactPort_MaskAxis(uint8_t axis){AppHardware_MaskLimitInterrupt(axis);}
void LimitSeek_ContactFromISR(uint8_t axis)
{
    uint32_t k=__get_PRIMASK();__disable_irq();
    /* GPIO配置时IRQ即可进入，此时App尚未初始化；端口直接停硬件且不依赖App状态。 */
    (void)SeekBoot_OnEdge(&boot_contact,trial.state==TRIAL_WAIT,axis,
        (uint8_t)((~GPIOC->IDR>>10)&7U),LimitIrqGuard_GetLatchMask(),HAL_GetTick());
    (void)SeekContact_OnEdge(&contact,trial.state==TRIAL_ACTIVE && armed,axis,
        (uint8_t)((~GPIOC->IDR>>10)&7U),LimitIrqGuard_GetLatchMask(),HAL_GetTick());
    __set_PRIMASK(k);
}
static void Cut(void)
{
    SeekContactPort_StopOutputs();
    if(RCC->APB1ENR&RCC_APB1ENR_TIM5EN){TIM5->CR1=0;TIM5->DIER=0;}
    __DSB();
}
static void Stop(TrialReason why)
{
    uint32_t k=__get_PRIMASK();__disable_irq();Cut();
    armed=false;
    if(why!=TRIAL_COMPLETE){
        Seek q=seek;Seek_Stop(&q,LimitIrqGuard_GetLatchMask()?SEEK_IRQ:SEEK_ABORT);seek=q;
    }
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
/* SysTick只执行有界纯状态机与GPIO写入；I2C/UART/打印全在主循环。 */
void __real_HAL_IncTick(void);
void __wrap_HAL_IncTick(void)
{
    __real_HAL_IncTick();uint32_t k=__get_PRIMASK();__disable_irq();
    if(limits_ready && trial.state==TRIAL_WAIT){
        for(uint8_t a=0;a<3;a++)LimitSeek_ContactFromISR(a);
        SeekBoot_Tick(&boot_contact,(uint8_t)((~GPIOC->IDR>>10)&7U),HAL_GetTick());
        if(boot_contact.failed){Seek q=seek;Seek_Stop(&q,SEEK_UNSTABLE);seek=q;Stop(TRIAL_APP_FAILED);}
    }
    Trial t=trial;bool was=t.state==TRIAL_ACTIVE;
    Trial_Tick(&t,HAL_GetTick(),main_seen);trial=t;
    if(was && t.state==TRIAL_ACTIVE && LimitIrqGuard_GetLatchMask()) {
        Seek q=seek;Seek_Stop(&q,SEEK_IRQ);seek=q;Stop(TRIAL_APP_FAILED);
    }
    if(trial.state==TRIAL_ACTIVE && armed) {
        /* GPIO轮询与ISR走同一入口；上电已压住或漏掉边沿也必须停输出再屏蔽。 */
        LimitSeek_ContactFromISR(seek.axis);
        Seek q=seek;
        /* 尚未轮到的启动已压触点变高，先停全部输出再确认，不用单采样判释放。 */
        uint8_t low=(uint8_t)((~GPIOC->IDR>>10)&7U);
        if((uint32_t)(HAL_GetTick()-feedback_ms)>=150U)Seek_Stop(&q,SEEK_FEEDBACK);
        else if(SeekHold_Service(&input_hold,&q,(uint8_t)(boot_contact.masked&~q.completed),low,HAL_GetTick()))
            Seek_Tick(&q,HAL_GetTick(),low);
        seek=q;
        for(unsigned a=0;a<3;a++)g_startup_trial.seek[a]=q.steps[a];
        if(q.state==SEEK_FAILED)Stop(TRIAL_APP_FAILED);
        else if(q.state==SEEK_DONE)Stop(TRIAL_COMPLETE);
#if STARTUP_TRIAL_MOTION_ENABLED
        else {
            /* 所有方向已在START前设置；只允许当前轴的1ms高脉冲。 */
            GPIOA->BSRR=(GPIO_PIN_6|GPIO_PIN_7)<<16;GPIOB->BSRR=GPIO_PIN_0<<16;
            GPIOD->BSRR=q.enable?(GPIO_PIN_2<<16):GPIO_PIN_2;
            if(q.enable && q.high) {
                if(q.axis<2U)GPIOA->BSRR=q.axis==0U?GPIO_PIN_6:GPIO_PIN_7;
                else GPIOB->BSRR=GPIO_PIN_0;
            }
        }
#endif
    }
    if(trial.state!=TRIAL_ACTIVE){if(was)Stop(trial.reason);else Cut();}
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
    /* APB1定时器84MHz，10kHz计数；32位TIM5按本候选总截止计数。 */
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
    if(actuator && trial.state!=TRIAL_ACTIVE){Cut();__set_PRIMASK(k);return;}
    __real_HAL_GPIO_WritePin(p,pin,value);
    __set_PRIMASK(k);
}
/* 本候选没有任何夹爪与正常回零入口；误调用同样切断全机。 */
HAL_StatusTypeDef __wrap_HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t channel)
{(void)t;(void)channel;Stop(TRIAL_OUTPUT);return HAL_ERROR;}
void __wrap_AppHardware_StartStepTimer(void){Stop(TRIAL_OUTPUT);}
bool __wrap_Dev_Joint_Step(DevJointId_t axis)
{(void)axis;Stop(TRIAL_OUTPUT);return false;}
void __real_AppSerial_SetMode(AppSerialMode_t);
void __wrap_AppSerial_SetMode(AppSerialMode_t mode)
{
    app_mode=mode;
    if(mode==APP_SERIAL_READY || mode==APP_SERIAL_FAILED) {
        Stop(TRIAL_APP_FAILED);mode=APP_SERIAL_OUTPUT_LOCKED;
    }
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
    if(Trial_IsQuery(out))return true;
    if(!strcmp(out,"M112")){Stop(TRIAL_HOST_STOP);puts("M112 OK\r");return false;}
    if(!strcmp(out,"M115")){
        printf("M115 FIRMWARE:RobotArmLimitSeek PROTOCOL:SEEK%u UART:115200 MOTION_COMPILED:%u AUTO_MOTION:0 SEEK_MAX:%u STEP_US:20000 DEADLINE_MS:%u BACKOFF:0,0,0 SEQUENCE:0,1,2 DIR:0,0,1 AXIS_TIMEOUT_MS:%u PRESSED_START:%u"
#if ROBOTARM_IRQ_EVIDENCE_MODE
            " FEATURES:IRQ_EVIDENCE"
#endif
            " CONTACT_DEBOUNCE_MS:10 CONTACT_TIMEOUT_MS:100 BOOT_CONTACT:1 INPUT_HOLD:1\r\n",6U,STARTUP_TRIAL_MOTION_ENABLED,
            SEEK_MAX_STEPS,TRIAL_DEADLINE_MS,SEEK_AXIS_TIMEOUT_MS,SEEK_ALLOW_PRESSED_START);return false;
    }
    if(!strcmp(out,"M125")) {
        uint32_t k=__get_PRIMASK();__disable_irq();Seek q=seek;uint8_t cm=contact.masked_mask,bm=boot_contact.masked,bs=boot_contact.stable;SeekInputHold h=input_hold;__set_PRIMASK(k);
        printf("M125 PHASE:%u REASON:%u AXIS:%u CONTACTS:%u STEPS:%lu,%lu,%lu IRQ:%u DRIVERS:%u CONTACT_IRQ_MASK:%u BOOT_IRQ_MASK:%u BOOT_STABLE_MASK:%u HOLD_STATE:%u HOLD_MASK:%u HOLD_INPUT:%u\r\n",
            q.state,q.reason,q.axis,q.completed,(unsigned long)q.steps[0],(unsigned long)q.steps[1],
            (unsigned long)q.steps[2],LimitIrqGuard_GetLatchMask(),drivers_prepared,cm,bm,bs,h.result,h.watch,h.input);return false;
    }
    if(!strcmp(out,"M123")){
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
        bool ready=initialized && drivers_prepared && app_mode==APP_SERIAL_CHECKING && !SafetyService_HasFault() &&
            !LimitIrqGuard_GetLatchMask() && feedback_seq && feedback_mask==7U &&
            (uint32_t)(HAL_GetTick()-feedback_ms)<150U &&
            (SEEK_ALLOW_PRESSED_START || ((~GPIOC->IDR>>10)&7U)==0U);
        uint32_t k=__get_PRIMASK();__disable_irq();Trial t=trial;
        ready=ready && !LimitIrqGuard_GetLatchMask() &&
            SeekBoot_Ready(&boot_contact,(uint8_t)((~GPIOC->IDR>>10)&7U)) &&
            (SEEK_ALLOW_PRESSED_START || ((~GPIOC->IDR>>10)&7U)==0U);
        bool ok=Trial_Start(&t,HAL_GetTick(),STARTUP_TRIAL_MOTION_ENABLED,ready);
        trial=t;main_seen=HAL_GetTick();if(ok)ArmDeadline();__set_PRIMASK(k);
        if(ok && !InitWatchdog()){Stop(TRIAL_OUTPUT);ok=false;}
        if(ok) {
            k=__get_PRIMASK();__disable_irq();Seek q=seek;
            ok=trial.state==TRIAL_ACTIVE && !LimitIrqGuard_GetLatchMask() &&
                Seek_Start(&q,HAL_GetTick(),(uint8_t)((~GPIOC->IDR>>10)&7U));
            seek=q;armed=ok;__set_PRIMASK(k);
            if(!ok)Stop(TRIAL_OUTPUT);
        }
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
    EarlyGPIO();HAL_Init();SystemClock_Config();MX_GPIO_Init();EarlyGPIO();limits_ready=true;
    MX_USART6_UART_Init();MX_USART1_UART_Init();MX_I2C1_Init();MX_I2C2_Init();MX_I2C3_Init();
    MX_TIM2_Init();MX_TIM6_Init();EarlyGPIO();
    /* 若原配置变化，不可把新参数冒充为本轮固定合同。 */
    if(HOMING_STEP_DELAY_US!=20000 || HOMING_MAX_STEPS!=200 || UNITS_PER_DEGREE!=40.0f ||
       HOMING_BACKOFF_M1_DEG!=100.0f || HOMING_BACKOFF_M2_DEG!=31.0f || HOMING_BACKOFF_M3_DEG!=5.0f ||
       HOMING_SEEK_M1_DIR_HIGH!=0 || HOMING_SEEK_M2_DIR_HIGH!=0 || HOMING_SEEK_M3_DIR_HIGH!=1)FailForever();
    App_Init();
    /* 正常RunOnce绝不调用：驱动确认可在EN禁用时完成，夹爪保持未初始化。 */
    uint8_t failed_node=0xff;drivers_prepared=AppHardware_PrepareMotionDrivers(&failed_node);
    for(unsigned a=0;a<3;a++)if(!Dev_Joint_SetDirectionLevel((DevJointId_t)a,a==2U))drivers_prepared=false;
    initialized=true;uint32_t last_feed=HAL_GetTick();
    while(1){
        main_seen=HAL_GetTick();
        if(watchdog_started && main_seen!=last_feed){IWDG->KR=0xaaaa;last_feed=main_seen;}
        uint32_t k=__get_PRIMASK();__disable_irq();bool clean=cleanup;cleanup=false;__set_PRIMASK(k);
        if(clean){
            AppHardware_StopActuators();Svc_Homing_Cancel();
            Seek_ReportTerminal(Snapshot().reason,seek.reason);
            __real_AppSerial_SetMode(APP_SERIAL_OUTPUT_LOCKED);
        }
        if(Snapshot().state==TRIAL_ACTIVE && LimitIrqGuard_GetLatchMask())Stop(TRIAL_APP_FAILED);
        SampleFeedback();AppSerial_Service();
        if(Snapshot().state!=TRIAL_ACTIVE)Cut();
    }
}
