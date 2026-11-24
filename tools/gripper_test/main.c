/** @file main.c
 * @brief 夹爪独立验证：步进EN永久禁用，只有显式动作构建允许一次600ms PWM。
 * @details 默认不初始化TIM2，PWM启动代码被预处理移除。保留115200查询及限位灯。
 * 正常App和全局安全许可均未链接；此工具不能宣称三轴HOMED或机构到位。
 */
#include "main.h"
#include "usart.h"
#include "driver/drv_servo.h"
#include "bsp_driver_config.h"
#include "gripper_logic.h"
#include "m1_limits.h"
#include "m1_rx.h"
#include "m1_deadline.h"
#include <stdio.h>
#include <string.h>
#ifndef GRIPPER_TEST_MOTION_ENABLED
#define GRIPPER_TEST_MOTION_ENABLED 0
#endif
#if GRIPPER_TEST_MOTION_ENABLED != 0 && GRIPPER_TEST_MOTION_ENABLED != 1
#error Invalid gripper motion permission
#endif
static GripTest test;
static volatile M1Rx rx;
static TIM_HandleTypeDef servo_timer;
static ServoDevice_t servo;
static volatile uint32_t main_seen;
static volatile bool cleanup;
static bool port_ready;
static void AxesOff(void)
{
    GPIOD->BSRR=GPIO_PIN_2;
    GPIOA->BSRR=(GPIO_PIN_6|GPIO_PIN_7)<<16;
    GPIOB->BSRR=GPIO_PIN_0<<16;
    __DSB();
}
static void CutPWM(void)
{
    AxesOff();
    if(RCC->APB1ENR&RCC_APB1ENR_TIM2EN){TIM2->CCER=0;TIM2->CR1=0;}
    GPIOA->BSRR=GPIO_PIN_1<<16;
    GPIOA->MODER=(GPIOA->MODER&~(3U<<2U))|(1U<<2U);
    TIM5->CR1=0;__DSB();
}
static M1LimitSnapshot Limits(void)
{return M1Limits_Decode(GPIOC->IDR,GPIOC->MODER,GPIOC->PUPDR);}
static void SafeGPIO(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_GPIOD_CLK_ENABLE();CutPWM();
    GPIO_InitTypeDef g={.Mode=GPIO_MODE_OUTPUT_PP,.Pull=GPIO_NOPULL,.Speed=GPIO_SPEED_FREQ_LOW};
    g.Pin=GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
    GPIOD->BSRR=GPIO_PIN_2|((GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5)<<16);HAL_GPIO_Init(GPIOD,&g);
    g.Pin=GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7;HAL_GPIO_Init(GPIOA,&g);
    g.Pin=GPIO_PIN_0;HAL_GPIO_Init(GPIOB,&g);
    GPIOC->BSRR=(GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3)|(GPIO_PIN_0<<16);
    g.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;HAL_GPIO_Init(GPIOC,&g);
    /* 只轮询：不继承正常App的EXTI锁存，也不把隔离策略称为中断根因修复。 */
    EXTI->IMR&=~0x1c00U;EXTI->RTSR&=~0x1c00U;EXTI->FTSR&=~0x1c00U;EXTI->PR=0x1c00U;
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    g.Pin=GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;g.Mode=GPIO_MODE_INPUT;g.Pull=GPIO_PULLUP;HAL_GPIO_Init(GPIOC,&g);
}
static GripTest Snapshot(void)
{uint32_t key=__get_PRIMASK();__disable_irq();GripTest t=test;__set_PRIMASK(key);return t;}
static void Stop(GripReason reason)
{
    uint32_t key=__get_PRIMASK();__disable_irq();
    bool active=test.state==GRIP_ACTIVE;CutPWM();GripTest_Stop(&test,reason);
    if(active)cleanup=true;
    __set_PRIMASK(key);
}
static void Tick(void)
{
    uint32_t key=__get_PRIMASK();__disable_irq();
    M1LimitSnapshot l=Limits();GPIOC->BSRR=M1Limits_LedBsrr(l.active_mask);
    bool active=test.state==GRIP_ACTIVE;
    GripTest_Tick(&test,HAL_GetTick(),main_seen,l.active_mask,l.config_ok);
    if(test.state!=GRIP_ACTIVE){CutPWM();if(active)cleanup=true;}else AxesOff();
    __set_PRIMASK(key);
}
void SysTick_Handler(void){HAL_IncTick();Tick();}
void TIM5_IRQHandler(void)
{
    if(M1Deadline_ExpiredFromISR()){
        /* 硬截止不依赖主循环；先切断PWM，再按同一输入快照结束窗口。 */
        CutPWM();Tick();
    }
}
void USART1_IRQHandler(void)
{
    uint32_t sr=USART1->SR;
    if(sr&(USART_SR_RXNE|USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)){
        uint8_t ch=(uint8_t)USART1->DR;
        if(sr&(USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)){
            M1Rx_Error(&rx);Stop(GRIP_SERIAL_ERROR);
        }else if(!M1Rx_Push(&rx,ch))Stop(GRIP_SERIAL_ERROR);
    }
}
void HardFault_Handler(void){CutPWM();while(1){}}
void MemManage_Handler(void){CutPWM();while(1){}}
void BusFault_Handler(void){CutPWM();while(1){}}
void UsageFault_Handler(void){CutPWM();while(1){}}
void Error_Handler(void){CutPWM();__disable_irq();while(1){}}
int _write(int fd,char *data,int size)
{
    (void)fd;
    if(HAL_UART_Transmit(&huart1,(uint8_t *)data,(uint16_t)size,100U)==HAL_OK)return size;
    Stop(GRIP_SERIAL_ERROR);return -1;
}
static void Clock(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitTypeDef o={0};o.OscillatorType=RCC_OSCILLATORTYPE_HSI;o.HSIState=RCC_HSI_ON;
    o.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;o.PLL.PLLState=RCC_PLL_ON;
    o.PLL.PLLSource=RCC_PLLSOURCE_HSI;o.PLL.PLLM=8;o.PLL.PLLN=168;o.PLL.PLLP=RCC_PLLP_DIV2;o.PLL.PLLQ=4;
    if(HAL_RCC_OscConfig(&o)!=HAL_OK)Error_Handler();
    RCC_ClkInitTypeDef c={0};c.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;c.AHBCLKDivider=RCC_SYSCLK_DIV1;c.APB1CLKDivider=RCC_HCLK_DIV4;c.APB2CLKDivider=RCC_HCLK_DIV2;
    if(HAL_RCC_ClockConfig(&c,FLASH_LATENCY_5)!=HAL_OK)Error_Handler();
}
static void InitStops(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();if(!M1Deadline_Prepare(GRIP_TEST_WINDOW_MS))Error_Handler();
    HAL_NVIC_SetPriority(TIM5_IRQn,0,0);HAL_NVIC_EnableIRQ(TIM5_IRQn);
    DBGMCU->APB1FZ&=~(DBGMCU_APB1_FZ_DBG_TIM5_STOP|DBGMCU_APB1_FZ_DBG_IWDG_STOP);
    RCC->CSR|=RCC_CSR_LSION;uint32_t began=HAL_GetTick();
    while(!(RCC->CSR&RCC_CSR_LSIRDY))if((uint32_t)(HAL_GetTick()-began)>100U)Error_Handler();
    IWDG->KR=0xcccc;IWDG->KR=0x5555;IWDG->PR=3;IWDG->RLR=499;
    began=HAL_GetTick();while(IWDG->SR)if((uint32_t)(HAL_GetTick()-began)>100U)Error_Handler();
    IWDG->KR=0xaaaa;
}
#if GRIPPER_TEST_MOTION_ENABLED
static void PreparePWM(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();servo_timer.Instance=TIM2;
    servo_timer.Init.Prescaler=83;servo_timer.Init.CounterMode=TIM_COUNTERMODE_UP;
    servo_timer.Init.Period=19999;servo_timer.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    servo_timer.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_PWM_Init(&servo_timer)!=HAL_OK)Error_Handler();
    TIM_OC_InitTypeDef c={.OCMode=TIM_OCMODE_PWM1,.Pulse=0,.OCPolarity=TIM_OCPOLARITY_HIGH,.OCFastMode=TIM_OCFAST_DISABLE};
    if(HAL_TIM_PWM_ConfigChannel(&servo_timer,&c,TIM_CHANNEL_2)!=HAL_OK)Error_Handler();
    GPIOA->AFR[0]=(GPIOA->AFR[0]&~(15U<<4U))|(1U<<4U);
    CutPWM();port_ready=true; /* PA1仍是低电平GPIO，不输出准备脉冲。 */
}
#endif
static void Start(GripTarget target)
{
#if GRIPPER_TEST_MOTION_ENABLED
    uint32_t key=__get_PRIMASK();__disable_irq();M1LimitSnapshot l=Limits();
    bool ok=M1Rx_DrainTail(&rx) && GripTest_Start(&test,HAL_GetTick(),target,true,port_ready,l.active_mask,l.config_ok);
    if(ok){
        main_seen=HAL_GetTick();
        if(!M1Deadline_Prepare(GRIP_TEST_WINDOW_MS))ok=false;
        if(ok){
            TIM2->CCR2=GripTest_TargetPulse(target);TIM2->EGR=TIM_EGR_UG;TIM2->SR=0;TIM2->CNT=0;
            TIM5->CR1=TIM_CR1_OPM|TIM_CR1_CEN;
            GPIOA->MODER=(GPIOA->MODER&~(3U<<2U))|(2U<<2U);
            ok=Drv_Servo_TrySetAngle(&servo,GripTest_TargetAngle(target));
        }
        if(!ok){CutPWM();GripTest_Stop(&test,GRIP_PWM_ERROR);cleanup=true;}
    }
    __set_PRIMASK(key);
    puts(ok?"accepted: one-shot gripper; lease required\r":"error: gripper start rejected\r");
#else
    (void)target;puts("error: compile locked\r");
#endif
}
static void Handle(const char *line)
{
    if(!strcmp(line,"M115")){
        printf("M115 FIRMWARE:GripperTest PROTOCOL:GRIP1 UART:115200 MOTION_COMPILED:%u AUTO_MOTION:0 OPEN_DEG:100 CLOSE_DEG:72 WINDOW_MS:600 JOINTS:LOCKED\r\n",GRIPPER_TEST_MOTION_ENABLED);return;
    }
    if(!strcmp(line,"M408")){
        GripTest t=Snapshot();bool pwm=(RCC->APB1ENR&RCC_APB1ENR_TIM2EN) && (TIM2->CR1&TIM_CR1_CEN) && (TIM2->CCER&TIM_CCER_CC2E);
        printf("M408 STATE:%u REASON:%u USED:%u PWM:%u TARGET:%u TARGET_DEG:%.3f PORT_READY:%u LIMIT_MASK:%u RXERR:%lu RXOVF:%lu EN:%s STEP:0 HOMED:0 ALLOWED:0\r\n",(unsigned)t.state,(unsigned)t.reason,t.used,pwm,(unsigned)t.target,GripTest_TargetAngle(t.target),port_ready,(unsigned)Limits().active_mask,(unsigned long)rx.uart_errors,(unsigned long)rx.overflows,(GPIOD->IDR&GPIO_PIN_2)?"DISABLED":"UNSAFE");return;
    }
    if(!strcmp(line,"M119")){
        unsigned m=Limits().active_mask;printf("M119 M1:%s M2:%s M3:%s\r\n",m&1?"TRIGGERED":"OPEN",m&2?"TRIGGERED":"OPEN",m&4?"TRIGGERED":"OPEN");return;
    }
    if(!strcmp(line,"KEEP")){
        uint32_t key=__get_PRIMASK();__disable_irq();bool ok=GripTest_Keep(&test,HAL_GetTick());__set_PRIMASK(key);
        puts(ok?"KEEP OK\r":"KEEP IDLE\r");return;
    }
    if(!strcmp(line,"M112")){Stop(GRIP_HOST_STOP);puts("M112OK\r");return;}
    if(!strcmp(line,"OPEN")){Start(GRIP_OPEN);return;}
    if(!strcmp(line,"CLOSE")){Start(GRIP_CLOSE);return;}
    Stop(GRIP_INPUT_FAULT);puts("error: command rejected\r");
}
int main(void)
{
    SafeGPIO();HAL_Init();Clock();GripTest_Init(&test);MX_USART1_UART_Init();SafeGPIO();
    if(GRIPPER_OPEN_ANGLE_DEG!=100.0f || GRIPPER_CLOSE_ANGLE_DEG!=72.0f || GRIPPER_HOLD_MS!=600U || GRIPPER_SERVO_MIN_US!=500.0f || GRIPPER_SERVO_MAX_US!=2500.0f)Error_Handler();
    if(!Drv_Servo_Bind(&servo,&servo_timer,TIM_CHANNEL_2))Error_Handler();
#if GRIPPER_TEST_MOTION_ENABLED
    PreparePWM();
#endif
    InitStops();setvbuf(stdout,NULL,_IONBF,0);
    HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);(void)USART1->SR;(void)USART1->DR;
    HAL_NVIC_SetPriority(USART1_IRQn,1,0);HAL_NVIC_EnableIRQ(USART1_IRQn);
    USART1->CR3|=USART_CR3_EIE;USART1->CR1|=USART_CR1_RXNEIE|USART_CR1_PEIE;
    char line[32];unsigned used=0;bool bad=false;uint32_t last_feed=HAL_GetTick();
    while(1){
        uint32_t now=HAL_GetTick();main_seen=now;AxesOff();
        if(now!=last_feed){IWDG->KR=0xaaaa;last_feed=now;}
        uint32_t key=__get_PRIMASK();__disable_irq();bool clean=cleanup;cleanup=false;
        bool fault=M1Rx_TakeFault(&rx);uint8_t ch=0;bool have=M1Rx_Pop(&rx,&ch);__set_PRIMASK(key);
#if GRIPPER_TEST_MOTION_ENABLED
        if(clean)(void)Drv_Servo_TryStop(&servo);
#else
        (void)clean;
#endif
        if(fault){Stop(GRIP_SERIAL_ERROR);used=0;bad=true;continue;}
        if(!have)continue;
        if(ch=='\r' || ch=='\n'){
            if(bad){Stop(GRIP_SERIAL_ERROR);puts("error: malformed line\r");}
            else if(used){line[used]='\0';Handle(line);}
            used=0;bad=false;
        }else if(ch<32 || ch>126 || used>=sizeof(line)-1U)bad=true;
        else if(!bad)line[used++]=(char)ch;
    }
}
