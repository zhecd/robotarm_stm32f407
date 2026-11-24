/**
 * @file guard.c
 * @brief 真实App启动审计的第二层输出锁及显式故障注入，不模拟机械成功。
 * @details 包装main只跳过TIM2/TIM6初始化，其余启动/设备/串口/状态机均生产源码。
 */
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"
#include "app_init.h"
#include "app_hardware_adapter.h"
#include "driver/drv_stepper.h"

/* 全为32位字段，供SWD记录。注入只把成功改为失败，不制造有效传感器数据。 */
volatile struct {
    uint32_t magic,blocked_outputs,prepare_calls,prepare_real_ok,prepare_forced_fail;
    uint32_t encoder_reads,encoder_actual_mask,inject_prepare_once,inject_encoder_mask;
} g_startup_audit={.magic=0x41554431U};
static void Inhibit(void)
{
    GPIOD->BSRR=GPIO_PIN_2;
    GPIOA->BSRR=(GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7)<<16;
    GPIOB->BSRR=GPIO_PIN_0<<16;
    __DSB();
}
static void EarlyGPIO(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_GPIOD_CLK_ENABLE();
    Inhibit();
    GPIO_InitTypeDef g={.Mode=GPIO_MODE_OUTPUT_PP,.Pull=GPIO_NOPULL,.Speed=GPIO_SPEED_FREQ_LOW};
    g.Pin=GPIO_PIN_2;HAL_GPIO_Init(GPIOD,&g);
    g.Pin=GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7;HAL_GPIO_Init(GPIOA,&g);
    g.Pin=GPIO_PIN_0;HAL_GPIO_Init(GPIOB,&g);
}
void __real_HAL_IncTick(void);
void __wrap_HAL_IncTick(void) { __real_HAL_IncTick();Inhibit(); }
void __real_HAL_GPIO_WritePin(GPIO_TypeDef *,uint16_t,GPIO_PinState);
void __wrap_HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,GPIO_PinState state)
{
    bool unsafe=(p==GPIOD && (pin&GPIO_PIN_2) && state==GPIO_PIN_RESET) ||
        (p==GPIOA && (pin&(GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7)) && state==GPIO_PIN_SET) ||
        (p==GPIOB && (pin&GPIO_PIN_0) && state==GPIO_PIN_SET);
    if (unsafe) { ++g_startup_audit.blocked_outputs;Inhibit();return; }
    __real_HAL_GPIO_WritePin(p,pin,state);
}
HAL_StatusTypeDef __wrap_HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t channel)
{ (void)t;(void)channel;++g_startup_audit.blocked_outputs;Inhibit();return HAL_ERROR; }
HAL_StatusTypeDef __wrap_HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *t)
{ (void)t;++g_startup_audit.blocked_outputs;Inhibit();return HAL_ERROR; }
bool __wrap_Drv_Stepper_Step(StepperDevice_t *m)
{ (void)m;++g_startup_audit.blocked_outputs;Inhibit();return false; }
bool __real_AppHardware_PrepareMotionDrivers(uint8_t *failed);
bool __wrap_AppHardware_PrepareMotionDrivers(uint8_t *failed)
{
    ++g_startup_audit.prepare_calls;
    Inhibit();
    if (g_startup_audit.inject_prepare_once) {
        g_startup_audit.inject_prepare_once=0;
        ++g_startup_audit.prepare_forced_fail;
        if (failed) *failed=1U;
        return false; /* 明确的软件注入：没有向真实驱动发送本次配置。 */
    }
    bool ok=__real_AppHardware_PrepareMotionDrivers(failed);
    if (ok) ++g_startup_audit.prepare_real_ok;
    return ok;
}
bool __real_AppHardware_ReadMotorAngle(uint8_t axis,float *out);
bool __wrap_AppHardware_ReadMotorAngle(uint8_t axis,float *out)
{
    ++g_startup_audit.encoder_reads;
    if (axis==0) g_startup_audit.encoder_actual_mask=0;
    bool ok=__real_AppHardware_ReadMotorAngle(axis,out);
    if (ok && axis<3) g_startup_audit.encoder_actual_mask|=1U<<axis;
    return ok && axis<3 && !(g_startup_audit.inject_encoder_mask&(1U<<axis));
}
void SystemClock_Config(void);
int __wrap_main(void)
{
    EarlyGPIO();HAL_Init();SystemClock_Config();MX_GPIO_Init();EarlyGPIO();
    MX_USART6_UART_Init();MX_USART1_UART_Init();
    MX_I2C1_Init();MX_I2C2_Init();MX_I2C3_Init();
    /* 不初始化TIM2/TIM6；真实App在首次夹爪/EN调用之前进入OUTPUT_LOCKED。 */
    App_Init();
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET); /* 保持存活LED常亮。 */
    while (1) { Inhibit();App_RunOnce(); }
}
