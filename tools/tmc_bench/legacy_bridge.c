/**
 * @brief 最初ZIP中的驱动+真实ST HAL桥接，不模拟HAL成功，不启动原应用。
 * 原文件逐字节保留在legacy_original；中文说明集中在本文件及阶段22记录。
 * 链接器wrap只记录真实HAL返回值和MCU输入读回，不冒充驱动端波形/电机反馈。
 */
#include "legacy_bridge.h"
#include "driver/drv_tmc2209.h"
#include "driver/drv_stepper.h"
#include <string.h>

static UART_HandleTypeDef uart6;
volatile LegacyBenchReport g_legacy_report;
HAL_StatusTypeDef __real_HAL_UART_Transmit(UART_HandleTypeDef *,const uint8_t *,uint16_t,uint32_t);
void __real_HAL_GPIO_WritePin(GPIO_TypeDef *,uint16_t,GPIO_PinState);

/* ST原HAL_Delay/HAL_UART_Transmit仍执行；只把毫秒源接到台架SysTick。 */
uint32_t HAL_GetTick(void) { return Bench_Milliseconds(); }
bool Legacy_Init(void)
{
    uart6.Instance=USART6; uart6.Init.BaudRate=115200;
    uart6.Init.WordLength=UART_WORDLENGTH_8B; uart6.Init.StopBits=UART_STOPBITS_1;
    uart6.Init.Parity=UART_PARITY_NONE; uart6.Init.Mode=UART_MODE_TX_RX;
    uart6.Init.HwFlowCtl=UART_HWCONTROL_NONE; uart6.Init.OverSampling=UART_OVERSAMPLING_16;
    /* 引脚/时钟已由台架初始化，HAL_UART_MspInit使用ST弱空实现，不开UART IRQ。 */
    if(HAL_UART_Init(&uart6)!=HAL_OK)return false;
    Drv_Stepper_Init();
    Drv_Stepper_SetDir(Drv_Stepper_GetM1(),STEPPER_DIRECTION_CW);
    g_legacy_report.initialized=1;return true;
}
bool Legacy_ConfigOriginal(void)
{
    g_legacy_report.tx_count=0;g_legacy_report.tx_fail=0;g_legacy_report.config_count=0;
    /* 原正常BSP轴表的M1参数：地址0、16微步、IRUN28、IHOLD15。 */
    Drv_TMC2209_ConfigNode(&uart6,0,16,28,15);
    return g_legacy_report.initialized && g_legacy_report.tx_count==3 &&
        g_legacy_report.tx_fail==0 && g_legacy_report.config_count==3 &&
        g_legacy_report.regs[0]==0 && g_legacy_report.values[0]==0xC1 &&
        g_legacy_report.regs[1]==0x6C && g_legacy_report.values[1]==0x14000053 &&
        g_legacy_report.regs[2]==0x10 && g_legacy_report.values[2]==0x61C0F;
}
void Legacy_Enable(void) { Drv_Stepper_EnableAll(true); }
void Legacy_Pulse(void)
{
    if(Drv_Stepper_Step(Drv_Stepper_GetM1()))++g_legacy_report.pulses;
}
HAL_StatusTypeDef __wrap_HAL_UART_Transmit(UART_HandleTypeDef *h,const uint8_t *p,uint16_t n,uint32_t timeout)
{
    HAL_StatusTypeDef result=__real_HAL_UART_Transmit(h,p,n,timeout);
    if(h==&uart6) {
        ++g_legacy_report.tx_count;
        if(result!=HAL_OK)++g_legacy_report.tx_fail;
        uint32_t i=g_legacy_report.config_count;
        if(n==8 && p[0]==5 && p[1]==0 && (p[2]&0x80) && i<3) {
            g_legacy_report.regs[i]=p[2]&0x7F;
            g_legacy_report.values[i]=((uint32_t)p[3]<<24)|((uint32_t)p[4]<<16)|((uint32_t)p[5]<<8)|p[6];
            g_legacy_report.config_count=i+1;
        }
    }
    return result; /* HAL_OK只是发送完成，不是TMC确认。 */
}
void __wrap_HAL_GPIO_WritePin(GPIO_TypeDef *port,uint16_t pin,GPIO_PinState state)
{
    __real_HAL_GPIO_WritePin(port,pin,state);
    if(port==GPIOA && pin==GPIO_PIN_6 && Bench_IsActive()) {
        uint32_t input=GPIOA->IDR;
        if(state==GPIO_PIN_SET) {
            if(input&GPIO_PIN_6)++g_legacy_report.high_seen;
            if(!(GPIOD->IDR&GPIO_PIN_2))++g_legacy_report.en_low_seen;
        } else if(!(input&GPIO_PIN_6))++g_legacy_report.low_seen;
    }
}
