/** 测试真实 UART BSP，仅 DMA 计数器与 HAL 收发替换为可控电脑端模型。 */
#include "usart.h"
#include "bsp/bsp_uart1.h"
#include "bsp/bsp_uart1_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static DMA_HandleTypeDef dma;
UART_HandleTypeDef huart1={USART1,&dma};
static uint8_t *rx;
static uint16_t size,position;
static uint8_t *tx;
static uint16_t tx_length;
static char wire[4096];
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t value) { (void)value; }
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *u,uint8_t *data,uint16_t count)
{ (void)u; rx=data; size=count; position=0; dma.counter=count; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) { (void)u; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,uint8_t *data,uint16_t count)
{ (void)u; tx=data; tx_length=count; return HAL_OK; }
static unsigned withheld_wraps;
static void FeedWithPolicy(const char *text,bool defer_wrap)
{
    while (*text) {
        rx[position++]=(uint8_t)*text++;
        if (position==size) {
            position=0;dma.counter=size;
            if(defer_wrap)withheld_wraps++;else HAL_UART_RxCpltCallback(&huart1);
        }
        else dma.counter=size-position;
    }
}
static void Feed(const char *text){FeedWithPolicy(text,false);}
static void Reset(void) { tick=0; tx_length=0; wire[0]=0; BSP_UART1_Init(); }
static void Drain(void)
{
    unsigned rounds=0;
    while (tx_length) {
        assert(rounds++<20);
        size_t length=strlen(wire);
        assert(length+tx_length<sizeof(wire));
        memcpy(wire+length,tx,tx_length); wire[length+tx_length]=0; tx_length=0;
        HAL_UART_TxCpltCallback(&huart1);
    }
}
int main(int argc,char **argv)
{
    char line[256]; char a[201],b[104]; memset(a,'A',200);a[200]=0; memset(b,'B',103);b[103]=0;
    if(argc==2) {
        Reset();char padding[255];memset(padding,'\n',254);padding[254]=0;
        Feed(padding);assert(!BSP_UART1_ReadLine(line,sizeof(line)));
        bool completed=!strcmp(argv[1],"completed_wrap");
        FeedWithPolicy("M119\n",!completed);
        if(completed)assert(BSP_UART1_ReadLine(line,sizeof(line)) && !strcmp(line,"M119"));
        else {
            if(!strcmp(argv[1],"discard_wrap"))BSP_UART1_DiscardRx();
            assert(!BSP_UART1_ReadLine(line,sizeof(line)));
            assert(!BSP_UART1_TakeRxOverflow());
            if(!strcmp(argv[1],"deferred_overwrite")) {
                char extra[257];memset(extra,'Z',256);extra[256]=0;FeedWithPolicy(extra,true);
            }
            while(withheld_wraps){HAL_UART_RxCpltCallback(&huart1);withheld_wraps--;}
            if(!strcmp(argv[1],"deferred_wrap"))assert(BSP_UART1_ReadLine(line,sizeof(line)) && !strcmp(line,"M119"));
            else if(!strcmp(argv[1],"discard_wrap")) {
                assert(!BSP_UART1_ReadLine(line,sizeof(line)));Feed("M408\n");
                assert(BSP_UART1_ReadLine(line,sizeof(line)) && !strcmp(line,"M408"));
            } else if(!strcmp(argv[1],"deferred_overwrite")) {
                assert(!BSP_UART1_ReadLine(line,sizeof(line)) && BSP_UART1_TakeRxOverflow());
            } else assert(false);
        }
        BspUartRxFault_t fault;BSP_UART1_GetRxFault(&fault);
        assert(fault.reason==(!strcmp(argv[1],"deferred_overwrite")?1U:0U));
        printf("bsp_uart_%s passed\n",argv[1]);return 0;
    }
    Reset(); Feed("M114\r\n"); assert(BSP_UART1_ReadLine(line,sizeof(line)) && strcmp(line,"M114")==0);
    assert(!BSP_UART1_ReadLine(line,sizeof(line)));
    Reset(); Feed("M114\n"); Feed(a); assert(BSP_UART1_ReadLine(line,sizeof(line)));
    Feed(b); assert(!BSP_UART1_ReadLine(line,sizeof(line)) && BSP_UART1_TakeRxOverflow());
    BspUartRxFault_t fault;BSP_UART1_GetRxFault(&fault);assert(fault.reason==1U);
    Feed("M3\nM119\n"); assert(BSP_UART1_ReadLine(line,sizeof(line)) && strcmp(line,"M119")==0);
    Reset(); Feed(a); assert(!BSP_UART1_ReadLine(line,sizeof(line)));
    Feed(b); Feed("M3\n"); assert(!BSP_UART1_ReadLine(line,sizeof(line)) && BSP_UART1_TakeRxOverflow());
    BSP_UART1_GetRxFault(&fault);assert(fault.reason==3U);
    Feed("M114\n"); assert(BSP_UART1_ReadLine(line,sizeof(line)) && strcmp(line,"M114")==0);
    Reset(); Feed("G1 X"); assert(!BSP_UART1_ReadLine(line,sizeof(line)));
    tick=501; assert(!BSP_UART1_ReadLine(line,sizeof(line)) && BSP_UART1_TakeLineTimeout());
    Feed("10\nM119\n"); assert(BSP_UART1_ReadLine(line,sizeof(line)) && strcmp(line,"M119")==0);
    Reset(); Feed("G1 X"); assert(!BSP_UART1_ReadLine(line,sizeof(line)));
    HAL_UART_ErrorCallback(&huart1); Feed("M3\nM114\n");
    assert(BSP_UART1_ReadLine(line,sizeof(line)) && strcmp(line,"M114")==0);
    BSP_UART1_GetRxFault(&fault);assert(fault.reason==2U);
    Reset(); BSP_UART1_SendString("M115 TEST\r\n");
    char huge[BSP_UART1_TX_BUF_SIZE]; memset(huge,'Z',sizeof(huge)-1);huge[sizeof(huge)-1]=0;
    BSP_UART1_SendString(huge); assert(BSP_UART1_TakeTxOverflow()); Drain();
    assert(strcmp(wire,"M115 TEST\r\n")==0); /* 空间不足时整条拒绝，不输出半条回复。 */
    puts("bsp_uart_tests: passed");return 0;
}
