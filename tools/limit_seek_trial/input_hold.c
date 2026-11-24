#include "input_hold.h"
#include "contact_gate.h"
bool SeekHold_Service(SeekInputHold *h,Seek *s,uint8_t watch,uint8_t input,uint32_t now)
{
    if(!h || !s || s->state==SEEK_FAILED || s->state==SEEK_DONE)return false;
    if(!h->paused){
        if(!(watch & (uint8_t)~input))return true;
        h->paused=true;h->result=HOLD_WAIT;h->watch=watch;h->began=now;
        h->high_seen=h->low_seen=0;
        s->enable=false;s->high=false;SeekContactPort_StopOutputs();
    }
    h->input=input;s->enable=false;s->high=false;
    /* 暂停不能延长原单轴截止；总截止、KEEP、反馈仍由外层先检查。 */
    if((uint32_t)(now-s->axis_ms)>=SEEK_AXIS_TIMEOUT_MS){Seek_Stop(s,SEEK_TIME);return false;}
    if((uint32_t)(now-h->began)>=100U){h->result=HOLD_UNSTABLE;Seek_Stop(s,SEEK_UNSTABLE);return false;}
    bool all_low=true;
    for(uint8_t a=0;a<3;a++){
        uint8_t bit=(uint8_t)(1U<<a);if(!(h->watch&bit))continue;
        if(!(input&bit)){
            all_low=false;h->low_seen&=(uint8_t)~bit;
            if(!(h->high_seen&bit)){h->high_seen|=bit;h->high_since[a]=now;}
            if((uint32_t)(now-h->high_since[a])>=10U){h->result=HOLD_RELEASED;Seek_Stop(s,SEEK_UNSTABLE);return false;}
        }else{
            h->high_seen&=(uint8_t)~bit;
            if(!(h->low_seen&bit)){h->low_seen|=bit;h->low_since[a]=now;}
            if((uint32_t)(now-h->low_since[a])<10U)all_low=false;
        }
    }
    if(!all_low)return false;
    h->paused=false;h->result=HOLD_RESUMED;
    /* 恢复后重新等20ms才发下一步，防止补发暂停期间积欠的步数。 */
    s->last_step_ms=now;s->enable=s->state==SEEK_MOVING;
    return true;
}
