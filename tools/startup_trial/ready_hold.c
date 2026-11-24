/** @file ready_hold.c
 * @brief 只在参考/许可持续有效且没有故障时累计保持窗口。
 */
#include "ready_hold.h"
bool ReadyHold_Arm(ReadyHold *s,uint32_t now,uint32_t duration_ms)
{
    if(!s || s->state!=READY_HOLD_IDLE || !duration_ms || duration_ms>10000U)return false;
    *s=(ReadyHold){READY_HOLD_RUNNING,now,duration_ms};return true;
}
ReadyHoldState ReadyHold_Tick(ReadyHold *s,uint32_t now,bool homed,bool allowed,bool fault)
{
    if(!s)return READY_HOLD_FAILED;
    if(s->state!=READY_HOLD_RUNNING)return s->state;
    /* 截止与故障同刻发生时，故障优先；终止后不允许重新装填。 */
    if(!homed || !allowed || fault)s->state=READY_HOLD_FAILED;
    else if((uint32_t)(now-s->began_ms)>=s->duration_ms)s->state=READY_HOLD_DONE;
    return s->state;
}
