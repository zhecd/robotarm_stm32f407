/* 真实BSP恢复流程；HAL替身模拟从机持低SDA及有界SCL等待，不代替电气验收。 */
#include "bsp/bsp_i2c_recovery.h"
#include "i2c.h"
#include <assert.h>
#include <stdio.h>
GPIO_TypeDef test_gpio_a,test_gpio_b,test_gpio_c;
I2C_HandleTypeDef hi2c1,hi2c2,hi2c3;
static GPIO_TypeDef *scl_port,*sda_port;
static uint16_t scl,sda;
static unsigned pulses,release_at,initializations,deinitializations,gpio_modes,delay_us;
static bool slave_holds_sda,scl_stuck,init_ok,deinit_ok;
static I2C_HandleTypeDef *selected;
static void Reset(unsigned axis)
{
    test_gpio_a=(GPIO_TypeDef){0,0xffff};test_gpio_b=(GPIO_TypeDef){1,0xffff};test_gpio_c=(GPIO_TypeDef){2,0xffff};
    hi2c1=hi2c2=hi2c3=(I2C_HandleTypeDef){HAL_I2C_STATE_READY,HAL_UNLOCKED,0};
    selected=axis==0?&hi2c1:axis==1?&hi2c2:&hi2c3;
    scl_port=axis==2?GPIOA:GPIOB;sda_port=axis==2?GPIOC:GPIOB;
    scl=axis==1?GPIO_PIN_10:GPIO_PIN_8;sda=axis==1?GPIO_PIN_11:GPIO_PIN_9;
    pulses=initializations=deinitializations=gpio_modes=delay_us=0;
    release_at=3;slave_holds_sda=true;scl_stuck=false;init_ok=deinit_ok=true;
}
void PlatformDelay_Us(uint32_t us) {delay_us+=us;assert(delay_us<20000);}
void HAL_GPIO_Init(GPIO_TypeDef *port,GPIO_InitTypeDef *config)
{
    assert((port==scl_port && config->Pin==scl)||(port==sda_port && config->Pin==sda));
    assert(config->Mode==GPIO_MODE_OUTPUT_OD && config->Pull==GPIO_PULLUP);
    assert(port->output&config->Pin); /* 输出模式切换前必须先预置释放。 */
    ++gpio_modes;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port,uint16_t pin,GPIO_PinState state)
{
    assert((port==scl_port && pin==scl)||(port==sda_port && pin==sda));
    bool was_high=(port->output&pin)!=0;
    if(state==GPIO_PIN_SET) port->output|=pin;else port->output&=(uint16_t)~pin;
    if(port==scl_port && pin==scl && !was_high && state==GPIO_PIN_SET && slave_holds_sda) {
        ++pulses;if(pulses>=release_at) slave_holds_sda=false;
    }
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port,uint16_t pin)
{
    if(port==scl_port && pin==scl && scl_stuck) return GPIO_PIN_RESET;
    if(port==sda_port && pin==sda && slave_holds_sda) return GPIO_PIN_RESET;
    return (port->output&pin)?GPIO_PIN_SET:GPIO_PIN_RESET;
}
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *bus)
{assert(bus==selected);++deinitializations;bus->State=HAL_I2C_STATE_RESET;return deinit_ok?HAL_OK:HAL_ERROR;}
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *bus)
{assert(bus==selected);++initializations;bus->State=HAL_I2C_STATE_READY;return init_ok?HAL_OK:HAL_ERROR;}
int main(void)
{
    for(unsigned axis=0;axis<3;axis++) {
        Reset(axis);assert(BSP_I2C_RecoverBus(selected));
        assert(pulses==3 && deinitializations==1 && initializations==1 && gpio_modes==2);
        assert((scl_port->output&scl) && (sda_port->output&sda));
    }
    Reset(1);release_at=9;assert(BSP_I2C_RecoverBus(selected) && pulses==9);
    Reset(1);release_at=100;assert(!BSP_I2C_RecoverBus(selected) && pulses==9 && initializations==1);
    Reset(1);scl_stuck=true;assert(!BSP_I2C_RecoverBus(selected));
    assert(pulses==0 && delay_us==1000 && initializations==1);
    Reset(1);slave_holds_sda=false;assert(BSP_I2C_RecoverBus(selected) && pulses==0);
    Reset(1);init_ok=false;assert(!BSP_I2C_RecoverBus(selected));
    Reset(1);deinit_ok=false;assert(!BSP_I2C_RecoverBus(selected) && gpio_modes==0);
    Reset(1);selected->Lock=HAL_LOCKED;assert(!BSP_I2C_RecoverBus(selected) && deinitializations==0);
    Reset(1);selected->State=HAL_I2C_STATE_BUSY_TX;assert(!BSP_I2C_RecoverBus(selected) && deinitializations==0);
    I2C_HandleTypeDef unknown={0};assert(!BSP_I2C_RecoverBus(NULL) && !BSP_I2C_RecoverBus(&unknown));
    puts("i2c_recovery_tests: passed");return 0;
}
