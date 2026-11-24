/** @file m1_rx.c @brief 主循环阻塞在I2C/打印时也保留收到的完整查询。 */
#include "m1_rx.h"
void M1Rx_Error(volatile M1Rx *r) { ++r->uart_errors;r->fault=true; }
bool M1Rx_Push(volatile M1Rx *r,uint8_t ch)
{
    if(r->fault)return false;
    uint16_t next=(uint16_t)((r->head+1U)%M1_RX_CAPACITY);
    if(next==r->tail){++r->overflows;r->fault=true;return false;}
    r->bytes[r->head]=ch;r->head=next;return true;
}
bool M1Rx_Pop(volatile M1Rx *r,uint8_t *ch)
{
    if(r->fault || r->head==r->tail)return false;
    *ch=r->bytes[r->tail];r->tail=(uint16_t)((r->tail+1U)%M1_RX_CAPACITY);return true;
}
bool M1Rx_TakeFault(volatile M1Rx *r)
{
    if(!r->fault)return false;
    r->tail=r->head;r->fault=false;return true;
}
bool M1Rx_DrainTail(volatile M1Rx *r)
{
    bool ok=!r->fault;uint8_t ch;
    while(M1Rx_Pop(r,&ch)) if(ch!='\r' && ch!='\n')ok=false;
    return ok;
}
