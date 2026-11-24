/** 中断接收与主循环延后消费的实际缓存回归，不模拟成功回复。 */
#include "m1_rx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(c) do{if(!(c)){fprintf(stderr,"line %d: %s\n",__LINE__,#c);exit(1);}}while(0)
int main(void)
{
    M1Rx r={0};uint8_t ch;
    /* 编码器阻塞期间到达完整请求：恢复后逐字节一致，无单字节DR覆盖。 */
    const char *requests="M114\r\nM119\nM408\r";
    for(unsigned cycle=0;cycle<1000;cycle++){
        for(size_t i=0;i<strlen(requests);i++)CHECK(M1Rx_Push(&r,(uint8_t)requests[i]));
        for(size_t i=0;i<strlen(requests);i++){CHECK(M1Rx_Pop(&r,&ch));CHECK(ch==(uint8_t)requests[i]);}
        CHECK(!M1Rx_Pop(&r,&ch));
    }
    for(unsigned i=0;i<M1_RX_CAPACITY-1;i++)CHECK(M1Rx_Push(&r,(uint8_t)i));
    CHECK(!M1Rx_Push(&r,'X'));CHECK(r.overflows==1 && r.fault);
    CHECK(!M1Rx_Pop(&r,&ch));CHECK(!M1Rx_Push(&r,'H'));
    CHECK(M1Rx_TakeFault(&r));CHECK(!M1Rx_TakeFault(&r));CHECK(!M1Rx_Pop(&r,&ch));
    CHECK(M1Rx_Push(&r,'M'));M1Rx_Error(&r);
    CHECK(!M1Rx_Pop(&r,&ch));CHECK(r.uart_errors==1);CHECK(!M1Rx_DrainTail(&r));
    CHECK(M1Rx_TakeFault(&r));CHECK(!M1Rx_Pop(&r,&ch));
    CHECK(M1Rx_Push(&r,'\r'));CHECK(M1Rx_Push(&r,'\n'));CHECK(M1Rx_DrainTail(&r));
    CHECK(M1Rx_Push(&r,'M'));CHECK(M1Rx_Push(&r,'1'));CHECK(!M1Rx_DrainTail(&r));
    CHECK(!M1Rx_Pop(&r,&ch));
    puts("m1_homing_rx_tests passed");return 0;
}
