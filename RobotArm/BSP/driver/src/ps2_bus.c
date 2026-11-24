/** @file ps2_bus.c
 * @brief PS2 GPIO时序；与模式协商/数据解释分开，保留原120循环的位时序。
 */
#include "driver/ps2_bus.h"
#include "main.h"

static void WritePin(GPIO_TypeDef *port,uint16_t pin,GPIO_PinState state)
{
    port->BSRR=state==GPIO_PIN_SET?(uint32_t)pin:((uint32_t)pin<<16U);
}
static void DelayBit(void)
{
    for(volatile uint32_t d=0;d<120U;d++){__NOP();}
}
static uint8_t Transfer(uint8_t tx)
{
    uint8_t rx=0U;
    for(uint8_t i=0;i<8U;i++) {
        WritePin(PS2_CMD_GPIO_Port,PS2_CMD_Pin,(tx&(1U<<i))?GPIO_PIN_SET:GPIO_PIN_RESET);
        WritePin(PS2_CLK_GPIO_Port,PS2_CLK_Pin,GPIO_PIN_RESET);DelayBit();
        if(PS2_DAT_GPIO_Port->IDR&PS2_DAT_Pin)rx|=(uint8_t)(1U<<i);
        WritePin(PS2_CLK_GPIO_Port,PS2_CLK_Pin,GPIO_PIN_SET);DelayBit();
    }
    DelayBit();return rx;
}
void Ps2Bus_Init(void)
{
    GPIO_InitTypeDef input={0};input.Pin=PS2_DAT_Pin;
    input.Mode=GPIO_MODE_INPUT;input.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(PS2_DAT_GPIO_Port,&input);
    WritePin(PS2_CLK_GPIO_Port,PS2_CLK_Pin,GPIO_PIN_SET);
    WritePin(PS2_CS_GPIO_Port,PS2_CS_Pin,GPIO_PIN_SET);
    WritePin(PS2_CMD_GPIO_Port,PS2_CMD_Pin,GPIO_PIN_SET);
}
void Ps2Bus_Exchange(const uint8_t *tx,uint8_t *rx,uint8_t length)
{
    /* 仅当前帧需要位时序原子性；跨帧等待由状态机处理，不在临界区延时毫秒。
     * 保存旧PRIMASK，调用方原先关中断时不能由此擅自开启。 */
    uint32_t mask=__get_PRIMASK();__disable_irq();
    WritePin(PS2_CS_GPIO_Port,PS2_CS_Pin,GPIO_PIN_RESET);DelayBit();
    for(uint8_t i=0;i<length;i++) {
        uint8_t value=Transfer(tx[i]);if(rx)rx[i]=value;
    }
    WritePin(PS2_CS_GPIO_Port,PS2_CS_Pin,GPIO_PIN_SET);__set_PRIMASK(mask);
}
