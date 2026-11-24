#include "boot_contact.h"
#include "contact_gate.h"
bool SeekBoot_OnEdge(SeekBootContact *b,bool waiting,uint8_t axis,uint8_t input,uint8_t latch,uint32_t now)
{
    if(!b || !waiting || b->failed || latch || axis>=3U)return false;
    uint8_t bit=(uint8_t)(1U<<axis);
    if(!(input&bit) || (b->masked&bit))return false;
    SeekContactPort_StopOutputs();SeekContactPort_MaskAxis(axis);
    b->masked|=bit;b->low_seen|=bit;b->began[axis]=b->low_since[axis]=now;
    return true;
}
void SeekBoot_Tick(SeekBootContact *b,uint8_t input,uint32_t now)
{
    if(!b || b->failed)return;
    for(unsigned a=0;a<3;a++){
        uint8_t bit=(uint8_t)(1U<<a);
        if(!(b->masked&bit))continue;
        if(b->stable&bit){
            if(!(input&bit)){
                /* 一次高采样只撤销START资格，不能冒充稳定释放。
                 * 固定新的100ms窗口重新确认；后续高采样不延长窗口。 */
                b->stable&=(uint8_t)~bit;b->low_seen&=(uint8_t)~bit;b->began[a]=now;
            }
            continue;
        }
        if((uint32_t)(now-b->began[a])>=100U){b->failed=true;continue;}
        if(!(input&bit)){b->low_seen&=(uint8_t)~bit;continue;}
        if(!(b->low_seen&bit)){b->low_seen|=bit;b->low_since[a]=now;}
        if((uint32_t)(now-b->low_since[a])>=10U)b->stable|=bit;
    }
}
bool SeekBoot_Ready(const SeekBootContact *b,uint8_t input)
{return b && !b->failed && b->stable==b->masked && (input&b->masked)==b->masked;}
bool SeekBoot_ReleasedBeforeContact(const SeekBootContact *b,uint8_t completed,uint8_t input)
{return b && (b->masked & (uint8_t)~(input|completed))!=0U;}
