/** @file limit_state_filter.c
 * @brief 正常运行限位消抖：首低暂停、稳定低锁存、稳定高取消短脉冲。
 * @details 每路单独记录首样本和稳定时间；重复调用不刷新总期限，缺失采样
 * 不算连续稳定。只输出状态，不控制电机；显式恢复前保持故障锁存。
 */
#include "common/limit_state_filter.h"
void LimitStateFilter_Reset(LimitStateFilter *s)
{if(s)*s=(LimitStateFilter){0};}
bool LimitStateFilter_BlocksSteps(const LimitStateFilter *s)
{return s && (s->pending || s->pressed || s->unstable);}
void LimitStateFilter_Sample(LimitStateFilter *s,uint32_t now,uint8_t low)
{
    if(!s)return;
    if(low&~7U){s->unstable=7U;return;}
    bool gap=s->sampled && (uint32_t)(now-s->last_ms)>1U;
    s->last_ms=now;s->sampled=true;
    for(uint8_t a=0;a<3U;a++){
        uint8_t bit=(uint8_t)(1U<<a);
        if((s->pressed|s->unstable)&bit)continue; /* 保留确认结果，显式恢复才复位。 */
        if(!(s->pending&bit)){
            if(!(low&bit))continue;
            s->pending|=bit;s->level_low|=bit;
            s->began[a]=s->stable_since[a]=now;
        }
        if(gap || ((low^s->level_low)&bit)){
            s->level_low=(uint8_t)((s->level_low&~bit)|(low&bit));
            s->stable_since[a]=now;
        }
        if((uint32_t)(now-s->stable_since[a])>=10U){
            if(low&bit)s->pressed|=bit;
            else s->pending&=(uint8_t)~bit;
        }else if((uint32_t)(now-s->began[a])>=100U)s->unstable|=bit;
    }
}
