#include "trial_logic.h"
#include <string.h>
bool Trial_Start(Trial *t,uint32_t now,bool compiled,bool ready)
{
    if(!t || !compiled || !ready || t->used || t->state!=TRIAL_WAIT)return false;
    t->used=true;t->state=TRIAL_ACTIVE;t->began_ms=now;t->keep_ms=now;return true;
}
void Trial_Stop(Trial *t,TrialReason why)
{
    if(!t || t->state==TRIAL_DONE || t->state==TRIAL_FAILED)return;
    t->used=true;t->reason=why;
    t->state=why==TRIAL_COMPLETE?TRIAL_DONE:TRIAL_FAILED;
}
bool Trial_Keep(Trial *t,uint32_t now)
{
    if(!t || t->state!=TRIAL_ACTIVE)return false;
    /* 迟到KEEP不能追认已过期租约。 */
    if((uint32_t)(now-t->keep_ms)>=TRIAL_LEASE_MS){Trial_Stop(t,TRIAL_LEASE_LOST);return false;}
    t->keep_ms=now;return true;
}
void Trial_Tick(Trial *t,uint32_t now,uint32_t main_seen)
{
    if(!t || t->state!=TRIAL_ACTIVE)return;
    if((uint32_t)(now-t->began_ms)>=TRIAL_DEADLINE_MS)Trial_Stop(t,TRIAL_DEADLINE);
    else if((uint32_t)(now-t->keep_ms)>=TRIAL_LEASE_MS)Trial_Stop(t,TRIAL_LEASE_LOST);
    else if((uint32_t)(now-main_seen)>=TRIAL_HEARTBEAT_MS)Trial_Stop(t,TRIAL_MAIN_STALLED);
}
bool Trial_IsQuery(const char *line)
{
    static const char *const queries[]={"M408","M409","M410","M411","M412","M114","M119"};
    if(!line)return false;
    for(unsigned i=0;i<sizeof(queries)/sizeof(queries[0]);++i)
        if(!strcmp(line,queries[i]))return true;
    return false;
}
