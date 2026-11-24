#include "trial_logic.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    Trial t={0};assert(!Trial_Start(&t,0,false,true));assert(!t.used);
    assert(!Trial_Start(&t,0,true,false));assert(Trial_Start(&t,0,true,true));
    assert(!Trial_Start(&t,1,true,true));
    Trial_Tick(&t,249,0);assert(t.state==TRIAL_ACTIVE);
    Trial_Tick(&t,250,0);assert(t.reason==TRIAL_MAIN_STALLED);
    assert(!Trial_Start(&t,251,true,true));assert(!Trial_Keep(&t,251));
    Trial_Stop(&t,TRIAL_COMPLETE);assert(t.reason==TRIAL_MAIN_STALLED);
    t=(Trial){0};assert(Trial_Start(&t,0,true,true));
    assert(Trial_Keep(&t,349));assert(!Trial_Keep(&t,699));assert(t.reason==TRIAL_LEASE_LOST);
    t=(Trial){0};uint32_t now=UINT32_MAX-100U;assert(Trial_Start(&t,now,true,true));
    for(uint32_t elapsed=100;elapsed<TRIAL_DEADLINE_MS;elapsed+=100){
        assert(Trial_Keep(&t,now+elapsed));Trial_Tick(&t,now+elapsed,now+elapsed);
        assert(t.state==TRIAL_ACTIVE);
    }
    Trial_Tick(&t,now+TRIAL_DEADLINE_MS,now+TRIAL_DEADLINE_MS);assert(t.reason==TRIAL_DEADLINE);
    t=(Trial){0};assert(Trial_Start(&t,0,true,true));Trial_Stop(&t,TRIAL_COMPLETE);
    assert(t.state==TRIAL_DONE && t.used);assert(!Trial_Start(&t,1,true,true));
    const char *bad[]={"M999","G1 X1","M3","M5","M119X"," M119","M119 ","M119\nSTART","M411 X1","M411\nSTART",""};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i)assert(!Trial_IsQuery(bad[i]));
    assert(Trial_IsQuery("M408") && Trial_IsQuery("M410") && Trial_IsQuery("M411") && !Trial_IsQuery(NULL));
    puts("startup trial one-shot/lease/deadline/wrap/whitelist passed");return 0;
}
