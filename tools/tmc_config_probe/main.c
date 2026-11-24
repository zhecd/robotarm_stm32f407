/**
 * @file main.c
 * @brief 独立TMC配置确认：真实生产驱动与BSP，所有命令始终禁止机械动作。
 * @details 不链接正常App/回零/夹爪/定时器启动；EN高、STEP低、PA1低。
 * 上电只初始化通信，不自动写驱动。M780单节点、M781真实BSP三节点确认。
 */
#include "main.h"
#include "usart.h"
#include "tim.h"
#include "bsp/bsp_robot_board.h"
#include "driver/drv_stepper.h"
#include "driver/drv_tmc2209.h"
#include "bsp_driver_config.h"
#include <stdio.h>
#include <string.h>

/* BSP中的未调用定时器端口没有初始化实例；不链接HAL_TIM或启动PWM。 */
TIM_HandleTypeDef htim6,htim2;
static volatile uint32_t violations;
static uint32_t runs;
static void Inhibit(void)
{
    GPIOD->BSRR=GPIO_PIN_2;
    GPIOA->BSRR=(GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7)<<16;
    GPIOB->BSRR=GPIO_PIN_0<<16;
    __DSB();
}
static void SafeGPIO(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_GPIOD_CLK_ENABLE();
    Inhibit();
    GPIO_InitTypeDef g={.Mode=GPIO_MODE_OUTPUT_PP,.Pull=GPIO_NOPULL,.Speed=GPIO_SPEED_FREQ_LOW};
    g.Pin=EN_Pin;HAL_GPIO_Init(GPIOD,&g);
    g.Pin=GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7;HAL_GPIO_Init(GPIOA,&g);
    g.Pin=GPIO_PIN_0;HAL_GPIO_Init(GPIOB,&g);
    g.Pin=GPIO_PIN_0;HAL_GPIO_Init(GPIOC,&g);
    GPIOC->BSRR=GPIO_PIN_0<<16; /* PC0低有效LED常亮。 */
}
void __real_HAL_GPIO_WritePin(GPIO_TypeDef *,uint16_t,GPIO_PinState);
void __wrap_HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,GPIO_PinState state)
{
    bool unsafe=(p==GPIOD && (pin&EN_Pin) && state==GPIO_PIN_RESET) ||
        (p==GPIOA && (pin&(GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7)) && state==GPIO_PIN_SET) ||
        (p==GPIOB && (pin&GPIO_PIN_0) && state==GPIO_PIN_SET);
    if (unsafe) { ++violations;Inhibit();return; }
    __real_HAL_GPIO_WritePin(p,pin,state);
}
void SysTick_Handler(void) { HAL_IncTick();Inhibit(); }
void HardFault_Handler(void) { Inhibit();while (1) {} }
void Error_Handler(void) { Inhibit();__disable_irq();while (1) {} }
int _write(int fd,char *data,int size)
{
    (void)fd;
    return HAL_UART_Transmit(&huart1,(uint8_t *)data,(uint16_t)size,200U)==HAL_OK?size:-1;
}
static void Clock(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitTypeDef o={0};
    o.OscillatorType=RCC_OSCILLATORTYPE_HSI;o.HSIState=RCC_HSI_ON;
    o.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;o.PLL.PLLState=RCC_PLL_ON;
    o.PLL.PLLSource=RCC_PLLSOURCE_HSI;o.PLL.PLLM=8;o.PLL.PLLN=168;
    o.PLL.PLLP=RCC_PLLP_DIV2;o.PLL.PLLQ=4;
    if (HAL_RCC_OscConfig(&o)!=HAL_OK) Error_Handler();
    RCC_ClkInitTypeDef c={0};
    c.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;c.AHBCLKDivider=RCC_SYSCLK_DIV1;
    c.APB1CLKDivider=RCC_HCLK_DIV4;c.APB2CLKDivider=RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&c,FLASH_LATENCY_5)!=HAL_OK) Error_Handler();
}
static void Status(void)
{
    printf("M408 EN:%s STEP:%lu PWM_CLOCK:%lu VIOLATIONS:%lu RUNS:%lu MS:%lu\r\n",
        (GPIOD->IDR&EN_Pin)?"DISABLED":"ERROR",
        (unsigned long)(((GPIOA->IDR>>6)&3U)|((GPIOB->IDR&1U)<<2)),
        (unsigned long)((RCC->APB1ENR&RCC_APB1ENR_TIM2EN)!=0U),
        (unsigned long)violations,(unsigned long)runs,(unsigned long)HAL_GetTick());
}
static void Handle(const char *s)
{
    Inhibit();
    if (!strcmp(s,"M115")) {
        puts("M115 FIRMWARE:TmcConfigProbe PROTOCOL:CFG1 UART:115200 MOTOR_ENABLE:LOCKED AUTO_MOTION:0\r");
    } else if (!strcmp(s,"M408")) Status();
    else if (strlen(s)==7 && !strncmp(s,"M780 N",6) && s[6]>='0' && s[6]<='3') {
        uint8_t n=(uint8_t)(s[6]-'0');TmcConfigReport_t r;
        bool ok=Drv_TMC2209_ConfigNodeVerified(&huart6,n,TMC_DEFAULT_MICROSTEPS,
            TMC_DEFAULT_IRUN,TMC_DEFAULT_IHOLD,&r);
        ++runs;Inhibit();
        printf("M780 NODE:%u VALID:%u RESULT:%u READ_ERROR:%u REG:%02X BEFORE:%u AFTER:%u EXPECTED:%08lX OBSERVED:%08lX\r\n",
            n,ok,r.result,r.read_error,r.failed_register,r.ifcnt_before,r.ifcnt_after,
            (unsigned long)r.expected,(unsigned long)r.observed);
    } else if (!strcmp(s,"M781")) {
        uint8_t failed;
        bool ok=BSP_RobotBoard_PrepareMotionDrivers(&failed);
        ++runs;Inhibit();printf("M781 VALID:%u FAILED_NODE:%u\r\n",ok,failed);
    } else if (strlen(s)==7 && !strncmp(s,"M122 N",6) && s[6]>='0' && s[6]<='3') {
        uint8_t n=(uint8_t)(s[6]-'0');
        const uint8_t regs[]={6,2,0,0x6c};uint32_t values[4]={0};
        TmcReadResult_t error=TMC_READ_OK;uint8_t failed=255;
        for (unsigned i=0;i<4;i++) {
            error=Drv_TMC2209_ReadRegisterResult(&huart6,n,regs[i],&values[i]);
            if (error!=TMC_READ_OK) { failed=regs[i];break; }
        }
        printf("M122 NODE:%u VALID:%u ERROR:%u REG:%02X IOIN:%08lX IFCNT:%lu GCONF:%08lX CHOP:%08lX\r\n",
            n,error==TMC_READ_OK,error,failed,(unsigned long)values[0],
            (unsigned long)values[1],(unsigned long)values[2],(unsigned long)values[3]);
    } else puts("error: configuration-probe command rejected\r");
}
int main(void)
{
    SafeGPIO();HAL_Init();Clock();Drv_Stepper_Init();Inhibit();
    MX_USART6_UART_Init();MX_USART1_UART_Init();
    /* USART1使用阻塞单字节接收，不启动DMA、UART中断或运动时钟。 */
    HAL_NVIC_DisableIRQ(USART1_IRQn);HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    setvbuf(stdout,NULL,_IONBF,0);
    char line[40];unsigned used=0;bool bad=false;uint8_t ch;
    while (1) {
        Inhibit();
        if (HAL_UART_Receive(&huart1,&ch,1,10U)!=HAL_OK) continue;
        if (ch=='\r' || ch=='\n') {
            if (bad) puts("error: line overflow\r");
            else if (used) { line[used]='\0';Handle(line); }
            used=0;bad=false;
        } else if (ch<32 || ch>126 || used>=sizeof(line)-1) bad=true;
        else if (!bad) line[used++]=(char)ch;
    }
}
