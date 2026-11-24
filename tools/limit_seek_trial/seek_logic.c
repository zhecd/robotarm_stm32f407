#include "seek_logic.h"
bool Seek_Start(Seek *s, uint32_t now, uint8_t limits)
{
    if (!s || s->used || s->state!=SEEK_WAIT || (limits&~7U)) return false;
    if (!SEEK_ALLOW_PRESSED_START && limits) return false;
    s->used=true; s->state=SEEK_MOVING; s->enable=true;
    s->initial_limits=limits;
    if(limits&1U)(void)Seek_BeginContact(s,now,0);
    s->axis_ms=now; s->last_step_ms=now; return true;
}
bool Seek_BeginContact(Seek *s, uint32_t now, uint8_t axis)
{
    if(!s || axis>=3U || axis!=s->axis)return false;
    if(s->state==SEEK_QUALIFY)return true;
    if(s->state!=SEEK_MOVING)return false;
    s->enable=false;s->high=false;s->state=SEEK_QUALIFY;
    s->edge_ms=now;s->qualify_ms=now;s->low_seen=true;return true;
}
void Seek_Stop(Seek *s, SeekReason reason)
{
    if (!s || s->state==SEEK_DONE || s->state==SEEK_FAILED) return;
    s->used=true; s->state=SEEK_FAILED; s->reason=reason;
    s->enable=false; s->high=false;
}
void Seek_Tick(Seek *s, uint32_t now, uint8_t limits)
{
    if (!s || s->state==SEEK_WAIT || s->state==SEEK_DONE || s->state==SEEK_FAILED) return;
    s->high=false;
    uint8_t own=(uint8_t)(1U<<s->axis);
    /* 已确认轴可以继续压着限位；尚未测试轴提前触发则停止整轮。 */
    if (limits & (uint8_t)~(s->completed|own|s->initial_limits)) { Seek_Stop(s,SEEK_UNEXPECTED); return; }
    if ((uint32_t)(now-s->axis_ms)>=SEEK_AXIS_TIMEOUT_MS) { Seek_Stop(s,SEEK_TIME); return; }
    if (s->state==SEEK_MOVING) {
        if (limits&own) {
            (void)Seek_BeginContact(s,now,s->axis);return;
        }
        if ((uint32_t)(now-s->last_step_ms)>=20U) {
            /* 步数只作第二道故障上限；扩展版正常终止依据是限位触发。 */
            if (s->steps[s->axis]>=SEEK_MAX_STEPS) { Seek_Stop(s,SEEK_BUDGET); return; }
            s->last_step_ms=now; ++s->steps[s->axis]; s->high=true;
        }
    } else if (s->state==SEEK_QUALIFY) {
        /* 消抖期间输出始终关闭。高电平重置连续低计时，不重新寻进。
         * 最多100ms，必须有连续10ms低采样；不能用一次短脉冲冒充到位。 */
        if((uint32_t)(now-s->qualify_ms)>=100U){Seek_Stop(s,SEEK_UNSTABLE);return;}
        if(!(limits&own)){s->low_seen=false;return;}
        if(!s->low_seen){s->low_seen=true;s->edge_ms=now;}
        if ((uint32_t)(now-s->edge_ms)>=10U) {
            s->completed|=own; s->edge_ms=now;
            s->state=s->axis==2U?SEEK_DONE:SEEK_GAP;
        }
    } else if (s->state==SEEK_GAP && (uint32_t)(now-s->edge_ms)>=100U) {
        ++s->axis; s->axis_ms=now; s->last_step_ms=now;
        /* 下一轴已压住时不使能，下一tick直接做稳定确认。 */
        s->enable=(limits&(1U<<s->axis))==0U; s->state=SEEK_MOVING;
    }
}
