/** 实际Homing→Device→Stepper→HAL GPIO链路，避免只测假CW参数掩盖极性反转。
 * 仅替换HAL、时间、限位和非方向外设端口；不模拟真实机械方向或脉宽。 */
#include "main.h"
#include "service/svc_homing.h"
#include "device/dev_joint.h"
#include "device/dev_limit_switch.h"
#include "driver/drv_stepper.h"
#include "driver/drv_as5600.h"
#include "bsp/bsp_robot_board.h"
#include "platform_critical.h"
#include "platform_time.h"
#include "limit_irq_guard.h"
#include "robot_config.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do {if(!(x)){fprintf(stderr,"GPIO contract line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
GPIO_TypeDef test_gpio_a,test_gpio_b,test_gpio_d;
static uint32_t now_us,depth;
static bool limits[3];
static unsigned gpio_writes,enable_low_writes,step_high_writes;
void HAL_GPIO_WritePin(GPIO_TypeDef *port,uint16_t pin,GPIO_PinState state)
{
    CHECK(port==&test_gpio_a || port==&test_gpio_b || port==&test_gpio_d);
    CHECK(state==GPIO_PIN_RESET || state==GPIO_PIN_SET);
    ++gpio_writes;
    if(port==&test_gpio_d && pin==(1U<<2) && state==GPIO_PIN_RESET)++enable_low_writes;
    if(state==GPIO_PIN_SET && ((port==&test_gpio_a && (pin==(1U<<6) || pin==(1U<<7))) || (port==&test_gpio_b && pin==1U)))++step_high_writes;
    if(state==GPIO_PIN_SET)port->ODR|=pin;else port->ODR&=~(uint32_t)pin;
}
void HAL_GPIO_Init(GPIO_TypeDef *port,GPIO_InitTypeDef *settings)
{CHECK(port!=NULL && settings!=NULL && settings->Mode==GPIO_MODE_OUTPUT_PP);}
uint32_t PlatformTime_NowUs(void){return now_us;}
uint32_t PlatformTime_NowMs(void){return now_us/1000U;}
void PlatformTime_Init(void){}
PlatformCriticalState_t PlatformCritical_Enter(void){return depth++;}
void PlatformCritical_Exit(PlatformCriticalState_t key){CHECK(depth==key+1);depth=key;}
uint8_t LimitIrqGuard_GetLatchMask(void){return 0;}
bool Dev_LimitSwitch_IsTriggered(DevJointId_t id){CHECK((unsigned)id<3);return limits[id];}

/* 这些接口为真实Device对象的非方向依赖，本测试不调用它们。 */
static As5600Device_t encoders[3];
As5600Device_t *Drv_AS5600_GetM1(void){return &encoders[0];}
As5600Device_t *Drv_AS5600_GetM2(void){return &encoders[1];}
As5600Device_t *Drv_AS5600_GetM3(void){return &encoders[2];}
ErrorCode_t Drv_AS5600_Init(void){return ERR_OK;}
ErrorCode_t Drv_AS5600_Update(As5600Device_t *enc){(void)enc;return ERR_ENCODER_FAIL;}
ErrorCode_t Drv_AS5600_SetZero(As5600Device_t *enc){(void)enc;return ERR_ENCODER_FAIL;}
void Drv_AS5600_PrintStatus(void){}
void BSP_RobotBoard_ConfigureMotionDrivers(void){}
bool BSP_RobotBoard_PrepareMotionDrivers(uint8_t *failed){if(failed)*failed=255;return false;}

static void CheckDir(unsigned id,bool high)
{
    /* 独立以实物PD3/PD4/PD5位号检查，不以fake_clockwise代替GPIO结果。 */
    CHECK(((test_gpio_d.ODR>>(id+3U))&1U)==(unsigned)high);
}
int main(void)
{
    test_gpio_a.ODR=0xA5001234U;test_gpio_b.ODR=0x5A004321U;test_gpio_d.ODR=0x1234ABCDU;
    Drv_Stepper_Init();CHECK(test_gpio_d.ODR&(1U<<2));
    for(unsigned id=0;id<3;id++) {
        /* 原兼容接口true=CW仍写低，false=CCW写高；不能为修回零而改变普通运动。 */
        CHECK(Dev_Joint_SetDirection((DevJointId_t)id,true));CheckDir(id,false);
        CHECK(Dev_Joint_SetDirection((DevJointId_t)id,false));CheckDir(id,true);
        CHECK(Dev_Joint_SetDirectionLevel((DevJointId_t)id,false));CheckDir(id,false);
        CHECK(Dev_Joint_SetDirectionLevel((DevJointId_t)id,true));CheckDir(id,true);
    }
    unsigned before=gpio_writes;
    CHECK(!Dev_Joint_SetDirectionLevel((DevJointId_t)3,true));
    CHECK(!Dev_Joint_SetDirectionLevel((DevJointId_t)-1,false));
    CHECK(gpio_writes==before && enable_low_writes==0 && step_high_writes==0);
    uint32_t untouched=test_gpio_d.ODR&~0x38U;
    Svc_Homing_Cancel();CHECK(Svc_Homing_Start());
    const bool expected[3]={HOMING_SEEK_M1_DIR_HIGH,HOMING_SEEK_M2_DIR_HIGH,HOMING_SEEK_M3_DIR_HIGH};
    for(unsigned id=0;id<3;id++)CheckDir(id,expected[id]);
#ifndef TEST_OVERRIDE_PHYSICAL_DIR
    CheckDir(0,false);CheckDir(1,false);CheckDir(2,true);
#else
    CheckDir(0,true);CheckDir(1,true);CheckDir(2,false);
#endif
    CHECK((test_gpio_d.ODR&~0x38U)==untouched && enable_low_writes==0 && step_high_writes==0);
    limits[0]=limits[1]=limits[2]=true;now_us+=HOMING_STEP_DELAY_US;Svc_Homing_Step();
    CHECK(Svc_Homing_GetState()==SVC_HOMING_BACKOFF);
    for(unsigned id=0;id<3;id++)CheckDir(id,!expected[id]);
    CHECK(enable_low_writes==0 && step_high_writes==0 && depth==0);
    Svc_Homing_Cancel();
    puts("homing_gpio_contract_tests: production Service/Device/Stepper GPIO chain passed; no physical motion tested");
    return 0;
}
