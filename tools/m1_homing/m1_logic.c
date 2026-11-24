/**
 * @file m1_logic.c
 * @brief 有限点动/寻限位/小退回；所有失败锁住会话，无自动重试或整机homed赋值。
 */
#include "m1_logic.h"
#include <stddef.h>

bool M1Test_Active(const M1Test *t)
{ return t && (t->state==M1_JOG || t->state==M1_SEEK || t->state==M1_QUALIFY || t->state==M1_BACKOFF); }
void M1Test_Init(M1Test *t) { if (t) *t=(M1Test){0}; }
bool M1Test_ConfirmDirection(M1Test *t,bool high)
{
    if (!t || t->used || M1Test_Active(t)) return false;
    t->direction_confirmed=true;t->seek_dir_high=high;return true;
}
void M1Test_Stop(M1Test *t,M1Reason reason)
{
    if (!t) return;
    t->used=true;t->enable=false;t->step=false;t->reason=reason;
    t->state=reason==M1_HOST_STOP?M1_STOPPED:M1_FAILED;
}
uint32_t M1Test_Deadline(const M1Test *t)
{ return t && t->state==M1_JOG?M1_JOG_DEADLINE_MS:M1_HOME_DEADLINE_MS; }
bool M1Test_Start(M1Test *t,uint32_t now,bool home,bool dir,bool permit,bool ready,const M1Inputs *in)
{
    if (!t || !in || !permit || !ready || t->used || M1Test_Active(t) ||
        !in->feedback_valid || in->m1_limit || in->other_limit ||
        (uint32_t)(now-in->main_seen_ms)>=M1_MAIN_STALE_MS ||
        (home && !t->direction_confirmed)) return false;
    t->used=true;t->state=home?M1_SEEK:M1_JOG;t->reason=M1_OK;
    t->dir_high=home?t->seek_dir_high:dir;t->step=false;t->enable=true;
    t->started=t->phase_started=t->last_rise=t->lease_ms=now;
    return true;
}
bool M1Test_KeepAlive(M1Test *t,uint32_t now)
{
    if (!M1Test_Active(t) || (uint32_t)(now-t->lease_ms)>=M1_LEASE_MS) return false;
    t->lease_ms=now;return true;
}
static void Complete(M1Test *t)
{ t->enable=false;t->step=false;t->state=M1_DONE;t->reason=M1_OK; }
void M1Test_Tick(M1Test *t,uint32_t now,const M1Inputs *in)
{
    if (!M1Test_Active(t)) return;
    if (!in) { M1Test_Stop(t,M1_INPUT_FAULT);return; }
    if ((uint32_t)(now-t->started)>=M1Test_Deadline(t)) { M1Test_Stop(t,M1_TIME_BOUND);return; }
    if ((uint32_t)(now-t->lease_ms)>=M1_LEASE_MS) { M1Test_Stop(t,M1_HOST_LOST);return; }
    if ((uint32_t)(now-in->main_seen_ms)>=M1_MAIN_STALE_MS) { M1Test_Stop(t,M1_MAIN_STALLED);return; }
    if (!in->feedback_valid) { M1Test_Stop(t,M1_FEEDBACK_LOST);return; }
    if (in->other_limit) { M1Test_Stop(t,M1_OTHER_LIMIT);return; }
    /* 每次先结束高电平。时间跳跃不会批量补步；物理脉宽仍须上板测量。 */
    if (t->step && (uint32_t)(now-t->last_rise)>=1U) t->step=false;
    if (t->state==M1_JOG && in->m1_limit) { M1Test_Stop(t,M1_LIMIT_DURING_JOG);return; }
    if (t->state==M1_SEEK && in->m1_limit) {
        t->step=false;t->state=M1_QUALIFY;t->limit_since=now;
    }
    if (t->state==M1_QUALIFY) {
        /* 第一条触发就停寻限位；稳定10ms后才准许反向，抖回直接失败。 */
        if (!in->m1_limit) { M1Test_Stop(t,M1_LIMIT_UNSTABLE);return; }
        if ((uint32_t)(now-t->limit_since)<M1_LIMIT_STABLE_MS) return;
        t->state=M1_BACKOFF;t->dir_high=!t->seek_dir_high;
        t->phase_started=t->last_rise=now;
        return; /* 重新等待2ms方向建立，绝不在切向同一tick发STEP。 */
    }
    if (t->state==M1_BACKOFF) {
        if (!in->m1_limit) {
            if (!t->release_candidate) { t->release_candidate=true;t->release_since=now; }
            if ((uint32_t)(now-t->release_since)>=M1_LIMIT_STABLE_MS) t->release_confirmed=true;
        } else {
            if (t->release_confirmed) { M1Test_Stop(t,M1_LIMIT_UNSTABLE);return; }
            t->release_candidate=false;
        }
        if (t->backoff_steps>=M1_BACKOFF_STEPS && !t->step) {
            if (t->release_confirmed) Complete(t);
            else if ((uint32_t)(now-t->last_rise)>=M1_BACKOFF_PERIOD_MS) M1Test_Stop(t,M1_LIMIT_STUCK);
            return;
        }
    }
    if (t->state==M1_JOG && t->jog_steps>=M1_JOG_STEPS && !t->step) { Complete(t);return; }
    if (t->state==M1_SEEK && t->seek_steps>=M1_SEEK_STEPS) { M1Test_Stop(t,M1_TRAVEL_BOUND);return; }
    uint32_t count=t->state==M1_JOG?t->jog_steps:t->state==M1_SEEK?t->seek_steps:t->backoff_steps;
    uint32_t period=t->state==M1_BACKOFF?M1_BACKOFF_PERIOD_MS:M1_PERIOD_MS;
    if ((uint32_t)(now-t->phase_started)<2U || t->step ||
        (count!=0U && (uint32_t)(now-t->last_rise)<period)) return;
    t->step=true;t->last_rise=now;
    if (t->state==M1_JOG) ++t->jog_steps;
    else if (t->state==M1_SEEK) ++t->seek_steps;
    else ++t->backoff_steps;
}
