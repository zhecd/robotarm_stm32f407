#include "driver/drv_tmc2209.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static USART_TypeDef registers;
static UART_HandleTypeDef uart={&registers};
static uint8_t tx[4][8],reply[8];
static unsigned tx_count,rx_count,fail_tx_at;
static HAL_StatusTypeDef rx_result;
static uint16_t last_size;
/* 测试侧先翻转每个字节，再用普通MSB优先CRC；与生产实现不同的路径。 */
static uint8_t ReferenceCRC(const uint8_t *bytes,unsigned size)
{
    uint8_t crc=0;
    for(unsigned i=0;i<size;i++) {
        uint8_t reversed=0;
        for(unsigned bit=0;bit<8;bit++) reversed|=(uint8_t)(((bytes[i]>>bit)&1U)<<(7U-bit));
        crc^=reversed;
        for(unsigned bit=0;bit<8;bit++) crc=(uint8_t)((crc<<1)^((crc&0x80U)?7U:0U));
    }
    return crc;
}
static uint32_t Word(unsigned index)
{
    return ((uint32_t)tx[index][3]<<24)|((uint32_t)tx[index][4]<<16)|
           ((uint32_t)tx[index][5]<<8)|tx[index][6];
}
static void Reset(void)
{
    registers.CR1=USART_CR1_RE;registers.DR=0;
    tx_count=rx_count=fail_tx_at=0;rx_result=HAL_OK;last_size=0;
    memset(tx,0,sizeof(tx));
    const uint8_t good[]={5,255,6,0x21,0,0,1,0};memcpy(reply,good,8);
    reply[7]=ReferenceCRC(reply,7);
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *handle,uint8_t *data,uint16_t size,uint32_t timeout)
{
    assert(handle==&uart && tx_count<4 && (size==4 || size==8) && timeout==100);
    assert(data[size-1]==ReferenceCRC(data,size-1));
    if(size==4) assert((registers.CR1&USART_CR1_RE)==0); /* 发送回声被抑制 */
    memcpy(tx[tx_count++],data,size);last_size=size;
    return tx_count==fail_tx_at?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *handle,uint8_t *data,uint16_t size,uint32_t timeout)
{
    assert(handle==&uart && size==8 && timeout==50 && last_size==4);
    assert((registers.CR1&USART_CR1_RE)!=0); /* 发送成功或失败后都恢复RE */
    ++rx_count;memcpy(data,reply,8);return rx_result;
}
void HAL_Delay(uint32_t ms) {assert(ms==2 || ms==5);}
int main(void)
{
    const uint16_t microsteps[]={256,128,64,32,16,8,4,2,1};
    for(unsigned m=0;m<9;m++) for(unsigned node=0;node<4;node++) {
        Reset();assert(Drv_TMC2209_ConfigNodeChecked(&uart,(uint8_t)node,microsteps[m],28,15));
        /* 三节点共用总线：必须先设置SENDDELAY=2，避免其他节点误判回复起始。 */
        assert(tx_count==4 && rx_count==0);
        assert(tx[0][0]==5 && tx[0][1]==node && tx[0][2]==0x83 && Word(0)==0x200U);
        assert(tx[1][2]==0x80 && Word(1)==0x1C1);
        assert(tx[2][2]==0xEC && Word(2)==(0x10000053U|(m<<24)));
        assert(tx[3][2]==0x90 && Word(3)==0x00061C0FU);
    }
    Reset();
    assert(!Drv_TMC2209_ConfigNodeChecked(NULL,0,16,1,1));
    UART_HandleTypeDef invalid={NULL};
    assert(!Drv_TMC2209_ConfigNodeChecked(&invalid,0,16,1,1));
    assert(!Drv_TMC2209_ConfigNodeChecked(&uart,4,16,1,1));
    assert(!Drv_TMC2209_ConfigNodeChecked(&uart,0,3,1,1));
    assert(!Drv_TMC2209_ConfigNodeChecked(&uart,0,16,32,1));
    assert(!Drv_TMC2209_ConfigNodeChecked(&uart,0,16,1,32) && tx_count==0);
    for(unsigned failure=1;failure<=4;failure++) {
        Reset();fail_tx_at=failure;
        assert(!Drv_TMC2209_ConfigNodeChecked(&uart,0,16,1,1) && tx_count==failure);
    }
    Reset();Drv_TMC2209_ConfigNode(&uart,0,16,1,1);assert(tx_count==4);
    for(unsigned node=0;node<4;node++) {
        Reset();uint32_t value=0;
        assert(Drv_TMC2209_ReadRegister(&uart,(uint8_t)node,6,&value));
        assert(tx_count==1 && rx_count==1 && last_size==4);
        assert(tx[0][0]==5 && tx[0][1]==node && tx[0][2]==6 && value==0x21000001U);
        if(node==0) assert(tx[0][3]==0x6FU); /* 固定IOIN请求校验向量 */
    }
    /* 回复帧每个字节损坏均被拒绝，且失败不覆盖调用方旧值。 */
    for(unsigned byte=0;byte<8;byte++) {
        Reset();reply[byte]^=0x01;uint32_t value=0x12345678U;
        assert(!Drv_TMC2209_ReadRegister(&uart,0,6,&value) && value==0x12345678U);
    }
    for(unsigned header=0;header<3;header++) {
        Reset();reply[header]^=0x01;reply[7]=ReferenceCRC(reply,7);uint32_t value=99;
        assert(!Drv_TMC2209_ReadRegister(&uart,0,6,&value) && value==99);
    }
    Reset();uint32_t value=99;rx_result=HAL_TIMEOUT;
    assert(!Drv_TMC2209_ReadRegister(&uart,0,6,&value) && value==99);
    Reset();fail_tx_at=1;
    assert(!Drv_TMC2209_ReadRegister(&uart,0,6,&value) && rx_count==0);
    assert((registers.CR1&USART_CR1_RE)!=0);
    Reset();
    assert(!Drv_TMC2209_ReadRegister(NULL,0,6,&value));
    assert(!Drv_TMC2209_ReadRegister(&invalid,0,6,&value));
    assert(!Drv_TMC2209_ReadRegister(&uart,4,6,&value));
    assert(!Drv_TMC2209_ReadRegister(&uart,0,0x80,&value));
    assert(!Drv_TMC2209_ReadRegister(&uart,0,6,NULL));
    registers.CR1=0;
    assert(!Drv_TMC2209_ReadRegister(&uart,0,6,&value) && tx_count==0);
    /* 详细结果接口定位哪一步失败，所有路径均保持失败前的输出值且不重试。 */
    Reset();value=99;
    assert(Drv_TMC2209_ReadRegisterResult(NULL,0,6,&value)==TMC_READ_ARGUMENT && value==99);
    registers.CR1=0;
    assert(Drv_TMC2209_ReadRegisterResult(&uart,0,6,&value)==TMC_READ_RX_DISABLED && tx_count==0);
    Reset();fail_tx_at=1;
    assert(Drv_TMC2209_ReadRegisterResult(&uart,0,6,&value)==TMC_READ_TX_FAILED && rx_count==0);
    assert((registers.CR1&USART_CR1_RE)!=0 && value==99);
    Reset();rx_result=HAL_TIMEOUT;
    assert(Drv_TMC2209_ReadRegisterResult(&uart,0,6,&value)==TMC_READ_RX_TIMEOUT && tx_count==1 && value==99);
    Reset();rx_result=HAL_ERROR;
    assert(Drv_TMC2209_ReadRegisterResult(&uart,0,6,&value)==TMC_READ_RX_FAILED && value==99);
    Reset();reply[0]^=1;reply[7]=ReferenceCRC(reply,7);
    assert(Drv_TMC2209_ReadRegisterResult(&uart,0,6,&value)==TMC_READ_FRAME && value==99);
    Reset();reply[7]^=1;
    assert(Drv_TMC2209_ReadRegisterResult(&uart,0,6,&value)==TMC_READ_CRC && value==99);
    Reset();reply[2]=0x6F;reply[3]=0xC0;reply[4]=0x1F;reply[5]=0;reply[6]=3;
    reply[7]=ReferenceCRC(reply,7);
    assert(Drv_TMC2209_ReadRegisterResult(&uart,0,0x6F,&value)==TMC_READ_OK);
    assert(value==0xC01F0003U && tx[0][2]==0x6F && tx_count==1 && rx_count==1);
    puts("tmc2209_driver_tests: passed");return 0;
}
