/**
 * @file bsp_i2c_recovery.c
 * @brief 编码器I2C总线解锁：开漏释放、最多9个恢复时钟、STOP及外设重初始化。
 */
#include "bsp/bsp_i2c_recovery.h"
#include "i2c.h"
#include "platform_delay.h"

typedef struct { GPIO_TypeDef *scl_port,*sda_port; uint16_t scl,sda; } BusPins_t;

static bool WaitHigh(GPIO_TypeDef *port,uint16_t pin)
{
    /* 有界轮询不依赖HAL毫秒时钟前进；开漏高表示释放，不能强推外部低电平。 */
    for (unsigned i=0;i<200;i++) {
        if (HAL_GPIO_ReadPin(port,pin)==GPIO_PIN_SET) return true;
        PlatformDelay_Us(5);
    }
    return false;
}
static void AsOpenDrain(GPIO_TypeDef *port,uint16_t pin)
{
    GPIO_InitTypeDef config={0};
    /* 先预置输出锁存为释放，再切到开漏输出，避免切换时意外拉低。 */
    HAL_GPIO_WritePin(port,pin,GPIO_PIN_SET);
    config.Pin=pin;config.Mode=GPIO_MODE_OUTPUT_OD;
    config.Pull=GPIO_PULLUP;config.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port,&config);
}

bool BSP_I2C_RecoverBus(I2C_HandleTypeDef *bus)
{
    BusPins_t pins;
    if (bus==&hi2c1) pins=(BusPins_t){GPIOB,GPIOB,GPIO_PIN_8,GPIO_PIN_9};
    else if (bus==&hi2c2) pins=(BusPins_t){GPIOB,GPIOB,GPIO_PIN_10,GPIO_PIN_11};
    else if (bus==&hi2c3) pins=(BusPins_t){GPIOA,GPIOC,GPIO_PIN_8,GPIO_PIN_9};
    else return false;
    if (bus->Lock!=HAL_UNLOCKED || bus->State!=HAL_I2C_STATE_READY) return false;
    if (HAL_I2C_DeInit(bus)!=HAL_OK) {
        (void)HAL_I2C_Init(bus);
        return false;
    }
    AsOpenDrain(pins.scl_port,pins.scl);
    AsOpenDrain(pins.sda_port,pins.sda);
    bool released=WaitHigh(pins.scl_port,pins.scl);
    for (unsigned i=0;released && i<9 && HAL_GPIO_ReadPin(pins.sda_port,pins.sda)==GPIO_PIN_RESET;i++) {
        HAL_GPIO_WritePin(pins.scl_port,pins.scl,GPIO_PIN_RESET);
        PlatformDelay_Us(5);
        HAL_GPIO_WritePin(pins.scl_port,pins.scl,GPIO_PIN_SET);
        released=WaitHigh(pins.scl_port,pins.scl);
        PlatformDelay_Us(5);
    }
    released=released && HAL_GPIO_ReadPin(pins.sda_port,pins.sda)==GPIO_PIN_SET;
    if (released) {
        /* SCL低时拉低SDA，随后先释放SCL、再释放SDA，形成合法STOP。 */
        HAL_GPIO_WritePin(pins.scl_port,pins.scl,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pins.sda_port,pins.sda,GPIO_PIN_RESET);
        PlatformDelay_Us(5);
        HAL_GPIO_WritePin(pins.scl_port,pins.scl,GPIO_PIN_SET);
        released=WaitHigh(pins.scl_port,pins.scl);
        PlatformDelay_Us(5);
        HAL_GPIO_WritePin(pins.sda_port,pins.sda,GPIO_PIN_SET);
        PlatformDelay_Us(5);
    }
    /* 每条退出路径都释放引脚并重新初始化，HAL MSP恢复原AF开漏及外设时序。
     * HAL_I2C_Init自身包含SWRST，不能只把软件ErrorCode清零冒充通信恢复。 */
    HAL_GPIO_WritePin(pins.scl_port,pins.scl,GPIO_PIN_SET);
    HAL_GPIO_WritePin(pins.sda_port,pins.sda,GPIO_PIN_SET);
    bool initialized=HAL_I2C_Init(bus)==HAL_OK;
    return released && initialized;
}
