/**
 * @file main.c
 * @brief 180专用串口自检：上电主动发送，随后每秒心跳，始终禁用执行器。
 * @details 仅CMSIS寄存器与SysTick；不用G-code、HAL发送中断或发送环。
 *          不解析输入，不启动业务、TMC、PS2、回零或PWM。
 *          与正式版相同的168MHz HSI/PLL和84MHz USART1时钟、PA9 AF7。
 */
#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdint.h>

/* SWD只读观察点：成功写入UART不代表电脑收到，不能据计数声称链路正常。 */
volatile struct {
    uint32_t magic;
    uint32_t ms;
    uint32_t clock_hz;
    uint32_t brr;
    uint32_t frames;
    uint32_t bytes;
    uint32_t tx_timeouts;
    uint32_t clock_error;
    uint32_t last_sr;
    uint32_t last_gpioa;
    uint32_t output_error;
} g_uart_boot_probe;

static bool SafeOutputs(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                     RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    (void)RCC->AHB1ENR;
    __DSB();
    /* 首次上电检查曾见PD2锁存为0；不只依赖时钟使能后的首次BSRR写入。
       增加总线同步和实际ODR/IDR检查，不将未核验的端口写入当作禁用成功。 */
    __NOP(); __NOP(); __NOP(); __NOP();
    /* PCB10_1/10_2：PD2低有效共用EN，先锁存高，再改输出，避免使能毛刺。 */
    GPIOD->BSRR = 1U << 2;
    GPIOD->MODER = (GPIOD->MODER & ~(3U << 4)) | (1U << 4);
    GPIOA->BSRR = ((1U << 6) | (1U << 7) | (1U << 1)) << 16;
    GPIOA->MODER = (GPIOA->MODER & ~((3U << 12) | (3U << 14) | (3U << 2))) |
                   (1U << 12) | (1U << 14) | (1U << 2);
    GPIOB->BSRR = 1U << 16;
    GPIOB->MODER = (GPIOB->MODER & ~3U) | 1U;
    /* STEP低、PA1低；不创建定时器，PWM不可能启动。禁用后重力下落需支撑。 */
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST | RCC_APB1RSTR_TIM6RST;
    RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM2RST | RCC_APB1RSTR_TIM6RST);
    GPIOC->BSRR = 0xfU;
    GPIOC->MODER = (GPIOC->MODER & ~0xffU) | 0x55U;
    GPIOC->BSRR = 1U << 16; /* LED1常亮；LED2每秒切换。 */
    GPIOD->BSRR = 1U << 2;
    __DSB();
    return (GPIOD->ODR & (1U << 2)) && (GPIOD->IDR & (1U << 2)) &&
           !(GPIOA->ODR & ((1U << 6) | (1U << 7) | (1U << 1))) &&
           !(GPIOB->ODR & 1U);
}

static bool WaitBits(volatile uint32_t *reg, uint32_t mask, uint32_t expected)
{
    /* 仅时钟初始化阶段使用有界自旋；失败保持输出禁用，不进入未知时钟发送。 */
    for (uint32_t n = 0; n < 4000000U; n++) {
        if ((*reg & mask) == expected) return true;
    }
    return false;
}

static bool Clock168MHz(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    PWR->CR |= PWR_CR_VOS;
    RCC->CR |= RCC_CR_HSION;
    if (!WaitBits(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY)) return false;
    RCC->PLLCFGR = 8U | (168U << 6) | (4U << 24);
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
    RCC->CFGR = RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CR |= RCC_CR_PLLON;
    if (!WaitBits(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY)) return false;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    if (!WaitBits(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL)) return false;
    SystemCoreClock = 168000000U;
    g_uart_boot_probe.clock_hz = SystemCoreClock;
    return true;
}

static void UartInit(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;
    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
    /* PA9发送；PA10不接管。LOW与正式固件一致，软件A/B已排除档位影响。 */
    GPIOA->MODER = (GPIOA->MODER & ~(3U << 18)) | (2U << 18);
    GPIOA->OTYPER &= ~(1U << 9);
    GPIOA->OSPEEDR &= ~(3U << 18);
    GPIOA->PUPDR &= ~(3U << 18);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(15U << 4)) | (7U << 4);
    USART1->BRR = (84000000U + 57600U) / 115200U;
    USART1->CR2 = 0U; /* 8N1，不反相，不使用流控、DMA或USART中断。 */
    USART1->CR3 = 0U;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
    g_uart_boot_probe.brr = USART1->BRR;
}

/* 主循环轮询发送；每字符/帧尾最多等20ms，失败记RAM并重试下一秒心跳。 */
static bool TxWait(uint32_t flag)
{
    uint32_t start = g_uart_boot_probe.ms;
    while (!(USART1->SR & flag)) {
        if ((uint32_t)(g_uart_boot_probe.ms - start) >= 20U) {
            g_uart_boot_probe.tx_timeouts++;
            return false;
        }
    }
    return true;
}

static bool Send(const char *text)
{
    for (; *text; text++) {
        if (!TxWait(USART_SR_TXE)) return false;
        USART1->DR = (uint8_t)*text;
        g_uart_boot_probe.bytes++;
    }
    if (!TxWait(USART_SR_TC)) return false;
    g_uart_boot_probe.frames++;
    return true;
}

static bool SendNumber(uint32_t number)
{
    char text[11];
    unsigned at = 10U;
    text[at] = '\0';
    do {
        text[--at] = (char)('0' + number % 10U);
        number /= 10U;
    } while (number);
    return Send(text + at);
}

void SysTick_Handler(void) { g_uart_boot_probe.ms++; }

int main(void)
{
    g_uart_boot_probe.magic = 0x55423180U;
    if (!SafeOutputs()) {
        g_uart_boot_probe.output_error = 1U;
        GPIOC->BSRR = 1U << 19;
        for (;;) __NOP(); /* 保持停止，不能带着未经确认的输出启动自检。 */
    }
    if (!Clock168MHz() || SysTick_Config(SystemCoreClock / 1000U)) {
        g_uart_boot_probe.clock_error = 1U;
        GPIOC->BSRR = 1U << 19; /* LED4常亮：时钟失败，等待SWD读取。 */
        for (;;) __NOP();
    }
    UartInit();
    (void)Send("\r\nBOOT UART_SELFTEST:180 BAUD:115200 FORMAT:8N1 TX:PA9 OUTPUTS:LOCKED\r\n");
    uint32_t last = g_uart_boot_probe.ms;
    uint32_t sequence = 0U;
    for (;;) {
        uint32_t now = g_uart_boot_probe.ms;
        if ((uint32_t)(now - last) >= 1000U) {
            last = now; /* 迟到不补发，始终约每秒一组，无运动业务。 */
            GPIOC->ODR ^= 1U << 1;
            (void)(Send("HEARTBEAT SEQ:") && SendNumber(++sequence) &&
                   Send(" UPTIME_MS:") && SendNumber(now) &&
                   Send(" OUTPUTS:LOCKED PATTERN:UUUUUUUU\r\n"));
            g_uart_boot_probe.last_sr = USART1->SR;
            g_uart_boot_probe.last_gpioa = GPIOA->IDR;
        }
        __WFI();
    }
}
