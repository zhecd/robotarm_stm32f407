/** 真实BSP停止端口：定时器初始化前的EXTI过载也不能解引用空Instance。 */
#include "main.h"
#include "bsp/bsp_robot_board.h"
#include "driver/drv_stepper.h"
#include "driver/drv_servo.h"
#include "driver/drv_tmc2209.h"
#include <assert.h>
#include <stdio.h>
TIM_HandleTypeDef htim6,htim2;
UART_HandleTypeDef huart6;
static unsigned timer_stops,pwm_stops,low_mask,disable_count;
static StepperDevice_t motors[3];
static unsigned prepared,fail_node=255;
static unsigned quiet_prepared,quiet_fail_node=255;
static unsigned recorded_stops;
static bool stop_ok=true,last_stopped;
int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *t) { (void)t;return 0; }
int HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *t) { assert(t->Instance==TIM6);++timer_stops;return 0; }
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *t,uint32_t ch)
{ assert(t->Instance==TIM2 && ch==TIM_CHANNEL_2);++pwm_stops;return stop_ok?HAL_OK:-1; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t pin) { (void)p;(void)pin;return GPIO_PIN_SET; }
bool Drv_Stepper_EnableAll(bool on) { assert(!on);++disable_count;return true; }
bool Drv_Stepper_WriteStepLevel(StepperDevice_t *m,bool high)
{ assert(!high && m>=motors && m<motors+3);low_mask|=1U<<(m-motors);return true; }
StepperDevice_t *Drv_Stepper_GetM1(void) { return &motors[0]; }
StepperDevice_t *Drv_Stepper_GetM2(void) { return &motors[1]; }
StepperDevice_t *Drv_Stepper_GetM3(void) { return &motors[2]; }
void Drv_TMC2209_ConfigNode(UART_HandleTypeDef *u,uint8_t a,uint16_t m,uint8_t r,uint8_t h)
{ (void)u;(void)a;(void)m;(void)r;(void)h; }
bool Drv_TMC2209_ConfigNodeVerified(UART_HandleTypeDef *u,uint8_t a,uint16_t m,
                                   uint8_t r,uint8_t h,TmcConfigReport_t *report)
{
    assert(u==&huart6 && a==prepared && m==16 && r==28 && h==15 && report);
    assert(disable_count>0);++prepared;
    /* 板级必须索取本次报告，不能只留下节点号。模拟三种不同的真实失败阶段，
     * 失败节点之后不得继续发送配置；成功路径仍不改变PWM/STEP生命周期。 */
    *report=(TmcConfigReport_t){.result=TMC_CONFIG_OK,.failed_register=0xFFU};
    if(a==fail_node){
        report->result=a==0?TMC_CONFIG_READ_FAILED:a==1?TMC_CONFIG_COUNTER_MISMATCH:TMC_CONFIG_VALUE_MISMATCH;
        report->read_error=a==0?TMC_READ_CRC:TMC_READ_OK;
        report->failed_register=a==2?0x6CU:0x02U;
        report->ifcnt_before=10;report->ifcnt_after=13;
        report->expected=a==1?4U:0x14000053U;report->observed=a==1?3U:0x10000053U;
    }
    return a!=fail_node;
}
bool Drv_TMC2209_ReadRegister(UART_HandleTypeDef *u,uint8_t a,uint8_t r,uint32_t *v)
{ (void)u;(void)a;(void)r;(void)v;return false; }
bool Drv_TMC2209_ConfigureQuietVerified(UART_HandleTypeDef *u,uint8_t a,TmcConfigReport_t *report)
{
    assert(u==&huart6 && a==quiet_prepared && prepared==a+1U && disable_count>0);
    ++quiet_prepared;*report=(TmcConfigReport_t){.result=TMC_CONFIG_OK,.failed_register=255};
    return a!=quiet_fail_node;
}
TmcReadResult_t Drv_TMC2209_ReadRegisterResult(UART_HandleTypeDef *u,uint8_t a,uint8_t r,uint32_t *v)
{ (void)u;(void)a;(void)r;(void)v;return TMC_READ_RX_TIMEOUT; }
ServoDevice_t *Drv_Servo_GetHandle(void) { return NULL; }
void Drv_Servo_Init(ServoDevice_t *s,TIM_HandleTypeDef *t,uint32_t c) { (void)s;(void)t;(void)c; }
bool Drv_Servo_TryInit(ServoDevice_t *s,TIM_HandleTypeDef *t,uint32_t c)
{ (void)s;(void)t;(void)c;return false; }
void Drv_Servo_RecordStopResult(ServoDevice_t *s,bool stopped)
{ assert(s==NULL);recorded_stops++;last_stopped=stopped; }
int main(void)
{
    BSP_RobotBoard_StopActuators();
    assert(timer_stops==0 && pwm_stops==0 && low_mask==7 && disable_count==1);
    htim6.Instance=TIM6;low_mask=0;
    BSP_RobotBoard_StopActuators();
    assert(timer_stops==1 && pwm_stops==0 && low_mask==7 && disable_count==2);
    htim2.Instance=TIM2;low_mask=0;
    BSP_RobotBoard_StopActuators();
    assert(timer_stops==2 && pwm_stops==1 && low_mask==7 && disable_count==3);
    uint8_t failed=99;
    assert(BSP_RobotBoard_PrepareMotionDrivers(&failed) && failed==255 && prepared==3);
    for (fail_node=0;fail_node<3;fail_node++) {
        prepared=quiet_prepared=0;
        assert(!BSP_RobotBoard_PrepareMotionDrivers(&failed));
        assert(failed==fail_node && prepared==fail_node+1); /* 首错停止。 */
    }
    prepared=quiet_prepared=0;fail_node=255;
    assert(BSP_RobotBoard_PrepareMotionDrivers(NULL) && prepared==3);
    assert(quiet_prepared==3);
    for(quiet_fail_node=0;quiet_fail_node<3;quiet_fail_node++) {
        prepared=quiet_prepared=0;
        assert(!BSP_RobotBoard_PrepareMotionDrivers(&failed));
        assert(failed==quiet_fail_node && prepared==quiet_fail_node+1 && quiet_prepared==prepared);
    }
    assert(timer_stops==2 && pwm_stops==1); /* 准备过程不改变夹爪生命周期。 */
    assert(recorded_stops==1 && last_stopped);
    stop_ok=false;BSP_RobotBoard_StopActuators();
    assert(recorded_stops==2 && !last_stopped && low_mask==7);
    puts("stop_actuators_tests: passed");return 0;
}
