/** @file gripper_logic.c @brief 单次窗口、租约和输入变化收尾，不存在自动重复或反向。 */
#include "gripper_logic.h"
void GripTest_Init(GripTest *t){if(t)*t=(GripTest){0};}
float GripTest_TargetAngle(GripTarget target)
{return target==GRIP_OPEN?100.0f:target==GRIP_CLOSE?72.0f:0.0f;}
uint32_t GripTest_TargetPulse(GripTarget target)
{return target==GRIP_OPEN?1611U:target==GRIP_CLOSE?1300U:0U;}
bool GripTest_Start(GripTest *t,uint32_t now,GripTarget target,bool permit,bool ready,
                    uint8_t limits,bool config_ok)
{
    if(!t || !permit || !ready || !config_ok || limits>7U || t->used ||
       t->state!=GRIP_IDLE || (target!=GRIP_OPEN && target!=GRIP_CLOSE))return false;
    t->used=true;t->state=GRIP_ACTIVE;t->target=target;t->reason=GRIP_OK;
    t->initial_limits=limits;t->started_ms=now;t->lease_ms=now;return true;
}
bool GripTest_Keep(GripTest *t,uint32_t now)
{
    if(!t || t->state!=GRIP_ACTIVE || (uint32_t)(now-t->lease_ms)>=GRIP_TEST_LEASE_MS ||
       (uint32_t)(now-t->started_ms)>=GRIP_TEST_WINDOW_MS)return false;
    t->lease_ms=now;return true;
}
void GripTest_Stop(GripTest *t,GripReason reason)
{
    if(!t)return;
    /* 收尾后重复停止不覆盖最早的失败原因；没有清USED的接口。 */
    if(t->state==GRIP_FAILED || t->state==GRIP_STOPPED)return;
    t->used=true;t->reason=reason;t->state=reason==GRIP_HOST_STOP?GRIP_STOPPED:GRIP_FAILED;
}
void GripTest_Tick(GripTest *t,uint32_t now,uint32_t main_seen,uint8_t limits,bool config_ok)
{
    if(!t || t->state!=GRIP_ACTIVE)return;
    if(!config_ok || limits>7U){GripTest_Stop(t,GRIP_INPUT_FAULT);return;}
    if(limits!=t->initial_limits){GripTest_Stop(t,GRIP_LIMIT_CHANGED);return;}
    if((uint32_t)(now-main_seen)>=GRIP_TEST_MAIN_MS){GripTest_Stop(t,GRIP_MAIN_STALE);return;}
    if((uint32_t)(now-t->lease_ms)>=GRIP_TEST_LEASE_MS){GripTest_Stop(t,GRIP_HOST_LOST);return;}
    if((uint32_t)(now-t->started_ms)>=GRIP_TEST_WINDOW_MS)t->state=GRIP_DONE;
}
