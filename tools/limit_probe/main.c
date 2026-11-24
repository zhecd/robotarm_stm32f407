/**
 * @file main.c
 * @brief PCB10_2最小GPIO/EXTI对照；RAM取证、可选轮询UART，始终禁用执行器。
 * @details 4个20秒阶段：检测关→pending轮询→直接IRQ→检测再关；完成后冻结统计。
 */
#include "stm32f4xx.h"
#include "probe_logic.h"
#include <stdbool.h>
#if LIMIT_PROBE_UART
#include "probe_console.h"
#include "bsp/uart_line_reader.h"
#endif
#define LIMIT_PINS (0x1c00U) /* PC10/11/12，与Core/Inc/main.h板级映射一致。 */
volatile ProbeState_t g_limit_probe;
volatile ProbeRecord_t g_limit_probe_trace[PROBE_TRACE_CAPACITY];

static bool WaitBits(volatile uint32_t *reg,uint32_t mask,uint32_t expected)
{
    for(uint32_t n=0;n<4000000U;n++)if((*reg&mask)==expected)return true;
    return false;
}
static void SafeOutputs(void)
{
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOBEN|
                    RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIODEN;
    (void)RCC->AHB1ENR;
    GPIOD->BSRR=1U<<2; /* PD2共用低有效EN：先写高，再设输出。 */
    GPIOD->MODER=(GPIOD->MODER&~(3U<<4))|(1U<<4);
    GPIOA->BSRR=((1U<<6)|(1U<<7))<<16;
    GPIOA->MODER=(GPIOA->MODER&~((3U<<12)|(3U<<14)))|(1U<<12)|(1U<<14);
    GPIOB->BSRR=1U<<16;
    GPIOB->MODER=(GPIOB->MODER&~3U)|1U;
    /* TIM2/TIM6保持复位态，不配置PWM、DIR、串口、I2C或PS2。 */
}
static bool Clock168MHz(void)
{
    /* 与正常固件相同：16MHz HSI /8 *168 /2，APB1/4、APB2/2。 */
    RCC->APB1ENR|=RCC_APB1ENR_PWREN;(void)RCC->APB1ENR;
    PWR->CR|=PWR_CR_VOS;
    RCC->CR|=RCC_CR_HSION;
    if(!WaitBits(&RCC->CR,RCC_CR_HSIRDY,RCC_CR_HSIRDY))return false;
    RCC->PLLCFGR=8U|(168U<<6)|(4U<<24);
    FLASH->ACR=FLASH_ACR_ICEN|FLASH_ACR_DCEN|FLASH_ACR_PRFTEN|FLASH_ACR_LATENCY_5WS;
    RCC->CFGR=RCC_CFGR_PPRE1_DIV4|RCC_CFGR_PPRE2_DIV2;
    RCC->CR|=RCC_CR_PLLON;
    if(!WaitBits(&RCC->CR,RCC_CR_PLLRDY,RCC_CR_PLLRDY))return false;
    RCC->CFGR=(RCC->CFGR&~RCC_CFGR_SW)|RCC_CFGR_SW_PLL;
    if(!WaitBits(&RCC->CFGR,RCC_CFGR_SWS,RCC_CFGR_SWS_PLL))return false;
    SystemCoreClock=168000000U;return true;
}
static uint32_t LowMask(void) { return (~GPIOC->IDR>>10)&7U; }
#if LIMIT_PROBE_UART
static UartLineReader_t s_uart_line;
static char s_tx[256];
static size_t s_tx_count,s_tx_at;
static void InitUart(void)
{
    /* APB2=84MHz，BRR四舍五入；轮询收发，不开DMA或USART1的NVIC通道。 */
    RCC->APB2ENR|=RCC_APB2ENR_USART1EN;(void)RCC->APB2ENR;
    GPIOA->MODER=(GPIOA->MODER&~((3U<<18)|(3U<<20)))|(2U<<18)|(2U<<20);
    GPIOA->AFR[1]=(GPIOA->AFR[1]&~((15U<<4)|(15U<<8)))|(7U<<4)|(7U<<8);
    GPIOA->OSPEEDR|=(3U<<18)|(3U<<20); /* 与原UART1的VERY_HIGH配置保持一致。 */
    USART1->BRR=(84000000U+57600U)/115200U;
    USART1->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;
    UartLineReader_Init(&s_uart_line);
}
static void PollUart(void)
{
    uint32_t status=USART1->SR;
    if(status&(USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) {
        (void)USART1->DR;UartLineReader_Invalidate(&s_uart_line);
    } else if(status&USART_SR_RXNE) {
        uint8_t byte=(uint8_t)USART1->DR;char line[32];
        if(UartLineReader_Feed(&s_uart_line,byte,g_limit_probe.ms,line,sizeof(line)) && !s_tx_count) {
            uint32_t key=__get_PRIMASK();__disable_irq();
            uint32_t phase=g_limit_probe.phase;
            ProbeConsoleSnapshot_t s={.phase=phase,.ms=g_limit_probe.ms,.low_mask=LowMask()};
            if(phase<PROBE_PHASE_COUNT)
                for(unsigned a=0;a<3;a++)s.pending[a]=g_limit_probe.stats[phase].pending[a];
            __set_PRIMASK(key);
            s_tx_count=ProbeConsole_Reply(line,&s,s_tx,sizeof(s_tx));s_tx_at=0;
        }
    }
    UartLineReader_CheckTimeout(&s_uart_line,g_limit_probe.ms,500U);
    if(s_tx_count && (USART1->SR&USART_SR_TXE)) {
        USART1->DR=(uint8_t)s_tx[s_tx_at++];
        if(s_tx_at==s_tx_count)s_tx_count=0;
    }
}
#endif
static void CapturePending(uint32_t source)
{
    uint32_t phase=g_limit_probe.phase;
    if(phase>=PROBE_PHASE_COUNT)return;
    uint32_t before=EXTI->PR,imr=EXTI->IMR;
    uint32_t handled=before&LIMIT_PINS;
    if(source==2U)handled&=imr;
    uint32_t gpio=GPIOC->IDR,cycles=DWT->CYCCNT;
    /* RM0090/原HAL均按位写1清pending，不读改写PR，不通过SWIER制造事件。 */
    if(handled)EXTI->PR=handled;
    __DSB();
    uint32_t after=EXTI->PR,gpio_after=GPIOC->IDR;
    uint32_t masked=ProbeLogic_Pending(&g_limit_probe.stats[phase],handled>>10,
                                      after>>10,g_limit_probe.ms,source==2U);
    if(source==2U)EXTI->IMR&=~(masked<<10);
    if(handled || source==2U) {
        uint32_t index=g_limit_probe.trace_count;
        /* 前阶段过密也给后阶段保留记录空间；数组仍按发布顺序连续。 */
        uint32_t phase_capacity=(phase+1U)*(PROBE_TRACE_CAPACITY/PROBE_PHASE_COUNT);
        if(index<phase_capacity) {
            ProbeRecord_t r={g_limit_probe.ms,phase,source,before,handled,after,gpio,gpio_after,
                             imr,EXTI->FTSR,EXTI->RTSR,SYSCFG->EXTICR[3],cycles};
            g_limit_probe_trace[index]=r;g_limit_probe.trace_count=index+1U;
        } else if(g_limit_probe.trace_dropped!=UINT32_MAX)g_limit_probe.trace_dropped++;
    }
}
static void SetPhase(uint32_t phase)
{
    uint32_t key=__get_PRIMASK();__disable_irq();
    NVIC_DisableIRQ(EXTI15_10_IRQn);
    EXTI->IMR&=~LIMIT_PINS;EXTI->RTSR&=~LIMIT_PINS;EXTI->FTSR&=~LIMIT_PINS;
    EXTI->PR=LIMIT_PINS;__DSB();NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    g_limit_probe.phase=phase;g_limit_probe.phase_start=g_limit_probe.ms;
    if(phase<PROBE_PHASE_COUNT)ProbeLogic_Init(&g_limit_probe.stats[phase],LowMask());
    if(phase==PROBE_POLL || phase==PROBE_IRQ)EXTI->FTSR|=LIMIT_PINS;
    if(phase==PROBE_IRQ) {
        EXTI->IMR|=LIMIT_PINS;NVIC_EnableIRQ(EXTI15_10_IRQn);
    }
    if(phase==PROBE_DONE)g_limit_probe.done=1U;
    __set_PRIMASK(key);
}
void SysTick_Handler(void) { g_limit_probe.ms++; }
void EXTI15_10_IRQHandler(void)
{
    if(g_limit_probe.phase==PROBE_IRQ)CapturePending(2U);
    else NVIC_DisableIRQ(EXTI15_10_IRQn);
}
int main(void)
{
    SafeOutputs();
    g_limit_probe.magic=0x4c505231U;g_limit_probe.version=1U;
    if(!Clock168MHz()) {
        g_limit_probe.clock_error=1U;g_limit_probe.done=1U;
        for(;;)__NOP(); /* 有界失败，继续保持EN禁用，等待只读取证。 */
    }
    g_limit_probe.clock_hz=SystemCoreClock;
    RCC->APB2ENR|=RCC_APB2ENR_SYSCFGEN;(void)RCC->APB2ENR;
    GPIOC->MODER&=~((3U<<20)|(3U<<22)|(3U<<24));
    GPIOC->PUPDR=(GPIOC->PUPDR&~((3U<<20)|(3U<<22)|(3U<<24)))|
                  (1U<<20)|(1U<<22)|(1U<<24);
    SYSCFG->EXTICR[2]=(SYSCFG->EXTICR[2]&~0xff00U)|0x2200U;
    SYSCFG->EXTICR[3]=(SYSCFG->EXTICR[3]&~0xfU)|2U;
    EXTI->EMR&=~LIMIT_PINS;
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0U;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
#if LIMIT_PROBE_UART
    InitUart();
#endif
    NVIC_SetPriority(EXTI15_10_IRQn,0U);
    SysTick_Config(SystemCoreClock/1000U);NVIC_SetPriority(SysTick_IRQn,1U);
    SetPhase(PROBE_OFF);g_limit_probe.ready=1U;
    uint32_t last_sample=UINT32_MAX;
    for(;;) {
#if LIMIT_PROBE_UART
        PollUart();
#endif
        uint32_t now=g_limit_probe.ms,phase=g_limit_probe.phase;
        /* 此板/探针在完成后WFI时曾返回全零RAM，暂停后可读回原数据。
         * 诊断结束保持活跃等待，避免把运行态调试读取限制误判为RAM丢失。 */
        if(phase==PROBE_DONE) { __NOP();continue; }
        if((uint32_t)(now-g_limit_probe.phase_start)>=PROBE_PHASE_MS) {
            SetPhase(phase+1U);continue;
        }
        if(now==last_sample)continue;
        last_sample=now;
        uint32_t key=__get_PRIMASK();__disable_irq();
        ProbeLogic_Sample(&g_limit_probe.stats[phase],LowMask());
        if(phase!=PROBE_IRQ && (EXTI->PR&LIMIT_PINS))CapturePending(1U);
        __set_PRIMASK(key);
    }
}
