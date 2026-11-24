/** @file home_sequence.c
 * @brief 回零纯算法：并行接触屏障/退回与逐轴对照，稳定释放后才提交完成。
 * @details 输入仅为毫秒时间及低有效位图，输出为STEP/DIR/EN和屏蔽/恢复请求。
 * 不操作外设，不延时，不打印；外层每1ms调用并串行化访问。
 * 首低样本暂停，低10ms确认接触，高10ms取消短脉冲，抖动100ms失败。
 * seek_timeout_ms为0仅取消单轴期限，整轮故障截止始终保留。
 */
#include "common/home_sequence.h"
#include <stddef.h>
#define STABLE_MS 10U
#define CONFIRM_MS 100U
#define GAP_MS 100U
static void ParallelTick(HomeSequence *s,uint32_t now,uint8_t low,bool gap);

void HomeSequence_Fail(HomeSequence *s,HomeReason reason)
{
    if(!s || s->phase==HOME_FAILED)return; /* 只保留首次原因。 */
    s->phase=HOME_FAILED;s->reason=reason;s->enable=false;s->high=false;s->step_mask=0;
    s->failure_low=s->last_low;s->failure_ms=s->last_sample_ms;
    s->rearm_request=0;
}
void HomeSequence_Init(HomeSequence *s,const HomeConfig *c)
{
    if(!s)return;
    *s=(HomeSequence){0};
    if(!c || c->period_ms<1U || c->period_ms>1000U || c->seek_dir>7U ||
       c->backoff_period_ms>1000U ||
       (c->seek_timeout_ms && c->seek_timeout_ms<100U) || c->seek_timeout_ms>60000U ||
       c->total_timeout_ms<100U || c->total_timeout_ms>600000U){
        HomeSequence_Fail(s,HOME_BAD_CONFIG);return;
    }
    for(unsigned a=0;a<3;a++)if(!c->backoff[a] || c->backoff[a]>100000U){
        HomeSequence_Fail(s,HOME_BAD_CONFIG);return;
    }
    s->config=*c;
    if(!s->config.backoff_period_ms)s->config.backoff_period_ms=c->period_ms;
}
static void Adopt(HomeSequence *s,uint8_t a,uint32_t now)
{
    uint8_t bit=(uint8_t)(1U<<a);
    s->enable=false;s->high=false;
    if(!(s->owned&bit)){
        s->owned|=bit;s->mask_request|=bit;
        s->boot_began[a]=s->boot_low_since[a]=now;s->boot_low_seen|=bit;
    }
}
void HomeSequence_PrepareRecovery(HomeSequence *s,const HomeConfig *c,uint8_t owned,uint32_t now)
{
    HomeSequence_Init(s,c);if(!s || s->phase==HOME_FAILED)return;
    s->recovery=true;s->owned=(uint8_t)(owned&7U);
    for(unsigned a=0;a<3;a++)s->boot_began[a]=now;
}
static void Contact(HomeSequence *s,uint32_t now)
{
    /* 已按下启动同样从当前位置退固定步数；不是边沿位置重建算法。
     * 退完仍低会在RELEASE失败，不能凭计数满足就判定参考建立。 */
    Adopt(s,s->axis,now);s->phase=HOME_CONTACT;s->phase_began=now;
    s->stable_since=now;s->low_seen=true;s->contact_high_seen=false;
}
bool HomeSequence_OnEdge(HomeSequence *s,uint8_t a,uint32_t now,uint8_t low)
{
    if(!s || a>=3U || !(low&(1U<<a)) || s->phase==HOME_FAILED)return false;
    s->last_low=low;s->last_sample_ms=now;
    if(s->phase==HOME_BOOT){Adopt(s,a,now);return true;}
    if(s->parallel && s->phase==HOME_SEEK){
        /* GPIO并行入口由1ms采样推进，外部事件最多取消本拍STEP，不在此发步。 */
        s->step_mask&=(uint8_t)~(1U<<a);s->high=s->step_mask!=0;return true;
    }
    if(s->phase==HOME_SEEK && a==s->axis){Contact(s,now);return true;}
    /* 已接管的触点不重复延长确认期限。退回重新布防后再触发则停机。 */
    if(s->owned&(1U<<a))return false;
    if(s->started){HomeSequence_Fail(s,HOME_UNEXPECTED);return true;}
    return false;
}
bool HomeSequence_BootReady(const HomeSequence *s,uint8_t low)
{
    return s && s->phase==HOME_BOOT && s->boot_stable==s->owned &&
           (low&s->owned)==s->owned;
}
bool HomeSequence_Start(HomeSequence *s,uint32_t now,uint8_t low)
{
    if(!HomeSequence_BootReady(s,low) || s->started)return false;
    s->started=true;s->initial=s->owned;s->began=s->axis_began=s->last_step=now;
    s->phase=HOME_SEEK;s->dir_high=(s->config.seek_dir&1U)!=0;s->enable=true;
    if(s->parallel){
        s->dir_mask=s->config.seek_dir;
        for(uint8_t a=0;a<3;a++){
            s->parallel_phase[a]=HOME_SEEK;s->parallel_last_step[a]=now;
        }
        ParallelTick(s,now,low,false);return s->phase!=HOME_FAILED;
    }
    if(low&1U)Contact(s,now);
    return true;
}
static void BootTick(HomeSequence *s,uint32_t now,uint8_t low)
{
    for(uint8_t a=0;a<3;a++){
        uint8_t bit=(uint8_t)(1U<<a);
        if((low&bit) && !(s->owned&bit))Adopt(s,a,now);
        if(!(s->owned&bit))continue;
        if(s->boot_stable&bit){
            if(!(low&bit)){
                s->boot_stable&=(uint8_t)~bit;s->boot_low_seen&=(uint8_t)~bit;
                s->boot_began[a]=now;
            }
            continue;
        }
        if((uint32_t)(now-s->boot_began[a])>=CONFIRM_MS){HomeSequence_Fail(s,HOME_UNSTABLE);return;}
        if(!(low&bit)){
            s->boot_low_seen&=(uint8_t)~bit;
            if(s->recovery){
                if(!(s->boot_high_seen&bit)){s->boot_high_seen|=bit;s->boot_low_since[a]=now;}
                if((uint32_t)(now-s->boot_low_since[a])>=STABLE_MS)s->rearm_request|=bit;
            }
            continue;
        }
        s->boot_high_seen&=(uint8_t)~bit;
        if(!(s->boot_low_seen&bit)){s->boot_low_seen|=bit;s->boot_low_since[a]=now;}
        if((uint32_t)(now-s->boot_low_since[a])>=STABLE_MS)s->boot_stable|=bit;
    }
}
/** 监视轴偏离期望电平时先停步。GPIO候选将未压轴的短低、已压轴的短高统一确认。
 * 持续异常10ms才报错，恢复期望电平10ms才续行；100ms仍不稳定则失败。 */
static bool HoldTick(HomeSequence *s,uint32_t now,uint8_t low)
{
    uint8_t watch=(uint8_t)(s->initial&~s->completed&~(1U<<s->axis));
    uint8_t expected=watch;
    if(s->config.qualify_unexpected){
        watch|=(uint8_t)((7U&~(s->initial|(1U<<s->axis)))|s->completed|s->rearmed);
    }
    if(s->parallel){watch=s->rearmed;expected=0U;}
    if(!s->holding){
        if(!((low^expected)&watch))return false;
        s->holding=true;s->hold_watch=watch;s->hold_began=now;
        s->hold_expect_low=expected;s->hold_sample_ms=now;
        s->hold_high_seen=s->hold_low_seen=0;s->hold_count++;
    }
    /* 并行退回的消抖暂停只停STEP，保留共用EN保持力，避免其他轴重力回落。 */
    s->enable=s->parallel;s->high=false;s->step_mask=0;
    if((uint32_t)(now-s->hold_began)>=CONFIRM_MS){HomeSequence_Fail(s,HOME_UNSTABLE);return true;}
    if(s->config.qualify_unexpected && (uint32_t)(now-s->hold_sample_ms)>1U){
        /* 未采样空档不算稳定；总确认期限仍从首次异常起计算。 */
        s->hold_high_seen=s->hold_low_seen=0;
    }
    s->hold_sample_ms=now;
    bool stable=true;
    for(uint8_t a=0;a<3;a++){
        uint8_t bit=(uint8_t)(1U<<a);if(!(s->hold_watch&bit))continue;
        /* high_seen/high_since沿用旧字段名，此处统一表示“偏离期望电平”。 */
        if((low^s->hold_expect_low)&bit){
            stable=false;s->hold_low_seen&=(uint8_t)~bit;
            if(!(s->hold_high_seen&bit)){s->hold_high_seen|=bit;s->hold_high_since[a]=now;}
            if((uint32_t)(now-s->hold_high_since[a])>=STABLE_MS){HomeSequence_Fail(s,HOME_UNEXPECTED);return true;}
        }else{
            s->hold_high_seen&=(uint8_t)~bit;
            if(!(s->hold_low_seen&bit)){s->hold_low_seen|=bit;s->hold_low_since[a]=now;}
            if((uint32_t)(now-s->hold_low_since[a])<STABLE_MS)stable=false;
        }
    }
    if(!stable)return true;
    uint32_t pause=(uint32_t)(now-s->hold_began);
    if(s->phase==HOME_BACKOFF || s->phase==HOME_RELEASE){
        if(s->back_pause_ms+pause>HOME_BACKOFF_PAUSE_MAX_MS){HomeSequence_Fail(s,HOME_UNSTABLE);return true;}
        s->back_pause_ms+=pause;
    }
    /* 平移暂停期间的时基，保留暂停前已等待的部分周期。
     * 不补发欠步：一次tick仍最多一脉冲，下一个脉冲仍间隔至少period_ms。 */
    s->holding=false;s->last_step+=pause;
    if(s->parallel)for(uint8_t a=0;a<3;a++)s->parallel_last_step[a]+=pause;
    s->enable=s->phase==HOME_SEEK || s->phase==HOME_BACKOFF;
    return false;
}
static void ReleaseTick(HomeSequence *s,uint32_t now,uint8_t low)
{
    uint8_t bit=(uint8_t)(1U<<s->axis);
    if(s->rearmed&bit){
        if((low&bit) && !s->config.qualify_unexpected)HomeSequence_Fail(s,HOME_UNEXPECTED);
        return;
    }
    if(low&bit){s->release_seen=false;return;}
    if(!s->release_seen){s->release_seen=true;s->stable_since=now;}
    if((uint32_t)(now-s->stable_since)>=STABLE_MS)s->rearm_request|=bit;
}
void HomeSequence_AckRearm(HomeSequence *s,uint8_t a,bool accepted)
{
    if(!s || a>=3U || !(s->rearm_request&(1U<<a)) || s->phase==HOME_FAILED)return;
    uint8_t bit=(uint8_t)(1U<<a);s->rearm_request&=(uint8_t)~bit;
    if(!accepted){HomeSequence_Fail(s,HOME_PORT);return;}
    s->owned&=(uint8_t)~bit;
    if(s->phase==HOME_BOOT)s->boot_stable&=(uint8_t)~bit;
    else s->rearmed|=bit;
}
void HomeSequence_Tick(HomeSequence *s,uint32_t now,uint8_t low)
{
    if(!s || s->phase==HOME_FAILED || s->phase==HOME_DONE)return;
    bool gap=s->started && (uint32_t)(now-s->last_sample_ms)>1U;
    if(s->config.qualify_unexpected && s->phase==HOME_CONTACT &&
       (uint32_t)(now-s->last_sample_ms)>1U){
        s->low_seen=false;s->contact_high_seen=false; /* 未采样空档不能算接触或释放稳定。 */
    }
    s->last_low=low;s->last_sample_ms=now;
    if(low&~7U){HomeSequence_Fail(s,HOME_UNEXPECTED);return;}
    s->high=false;
    if(s->phase==HOME_BOOT){BootTick(s,now,low);return;}
    if((uint32_t)(now-s->began)>=s->config.total_timeout_ms){HomeSequence_Fail(s,HOME_TOTAL_TIMEOUT);return;}
    if(s->parallel){ParallelTick(s,now,low,gap);return;}
    uint8_t own=(uint8_t)(1U<<s->axis);
    if(!s->config.qualify_unexpected){
        if(low & (uint8_t)~(s->initial|own)){HomeSequence_Fail(s,HOME_UNEXPECTED);return;}
        if(low&s->completed){HomeSequence_Fail(s,HOME_UNEXPECTED);return;}
    }
    if(s->config.seek_timeout_ms && (s->phase==HOME_SEEK || s->phase==HOME_CONTACT) &&
       (uint32_t)(now-s->axis_began)>=s->config.seek_timeout_ms){HomeSequence_Fail(s,HOME_SEEK_TIMEOUT);return;}
    if(s->phase==HOME_BACKOFF || s->phase==HOME_RELEASE){
        uint32_t pause=s->back_pause_ms;
        if(s->holding)pause+=(uint32_t)(now-s->hold_began);
        if(pause>HOME_BACKOFF_PAUSE_MAX_MS){HomeSequence_Fail(s,HOME_UNSTABLE);return;}
        if((uint32_t)(now-s->phase_began)>=s->config.backoff[s->axis]*s->config.backoff_period_ms+500U+pause){
            HomeSequence_Fail(s,HOME_BACK_TIMEOUT);return;
        }
    }
    if(s->phase==HOME_SEEK && (low&own))Contact(s,now);
    if(HoldTick(s,now,low)){
        if(s->config.qualify_unexpected && s->phase==HOME_CONTACT){
            s->low_seen=false;s->contact_high_seen=false;
        }
        return;
    }
    if(s->phase==HOME_SEEK){
        s->enable=true;
        if((uint32_t)(now-s->last_step)>=s->config.period_ms){
            s->last_step=now;s->seek[s->axis]++;s->high=true;
        }
    }else if(s->phase==HOME_CONTACT){
        s->enable=false;
        if((uint32_t)(now-s->phase_began)>=CONFIRM_MS){HomeSequence_Fail(s,HOME_UNSTABLE);return;}
        if(!(low&own)){
            s->low_seen=false;
            if(s->config.qualify_unexpected){
                if(!s->contact_high_seen){s->contact_high_seen=true;s->contact_high_since=now;}
                if((uint32_t)(now-s->contact_high_since)>=STABLE_MS){
                    /* 首低已停步；高稳定10ms说明未确认接触，取消短脉冲。
                     * 恢复原寻程，不重置轴/整轮期限，不补发脉冲，再等完整寻位步周期。 */
                    s->phase=HOME_SEEK;s->enable=true;s->last_step=now;s->contact_high_seen=false;
                    if(s->contact_cancel_count<UINT32_MAX)s->contact_cancel_count++;
                }
            }
            return;
        }
        s->contact_high_seen=false;
        if(!s->low_seen){s->low_seen=true;s->stable_since=now;}
        if((uint32_t)(now-s->stable_since)>=STABLE_MS){
            s->contacts|=own;s->phase=HOME_BACKOFF;s->phase_began=s->last_step=now;
            s->dir_high=(s->config.seek_dir&own)==0;s->enable=true;s->release_seen=false;s->back_pause_ms=0;
        }
    }else if(s->phase==HOME_BACKOFF){
        ReleaseTick(s,now,low);if(s->phase==HOME_FAILED)return;
        if(s->back[s->axis]>=s->config.backoff[s->axis]){
            s->enable=false;s->phase=HOME_RELEASE;return;
        }
        s->enable=true;
        /* 退回独立定速。1ms最短周期每拍一个请求；真实端口先拉低再输出上升沿。
         * 纯算法不承诺实际脉宽，同毫秒重复调用不重复发步。 */
        if((uint32_t)(now-s->last_step)>=s->config.backoff_period_ms){
            s->last_step=now;s->back[s->axis]++;s->high=true;
        }
    }else if(s->phase==HOME_RELEASE){
        ReleaseTick(s,now,low);if(s->phase==HOME_FAILED)return;
        if(s->rearmed&own){
            s->completed|=own;s->phase=s->axis==2U?HOME_DONE:HOME_GAP;
            s->stable_since=now;s->enable=false;
        }
    }else if(s->phase==HOME_GAP && (uint32_t)(now-s->stable_since)>=GAP_MS){
        s->axis++;s->phase=HOME_SEEK;s->axis_began=s->last_step=now;
        s->dir_high=(s->config.seek_dir&(1U<<s->axis))!=0;s->enable=true;
        if(low&(1U<<s->axis))Contact(s,now);
    }
}

/** 每轴独立采样及定速发步，不追补错过的周期。先到限位的轴保持使能但不发STEP。
 * 并行寻位屏障要求三轴此刻均连续低10ms；仅有历史CONTACTS:7不允许反转。
 * 退回各自达到配置步数且稳定释放并获端口回执后才完成，任何失败停全部输出。 */
static void ParallelTick(HomeSequence *s,uint32_t now,uint8_t low,bool gap)
{
    s->step_mask=0;s->high=false;s->enable=true;
    if(gap){s->parallel_low_seen=0;s->parallel_high_seen=0;}
    if(s->phase==HOME_SEEK){
        uint8_t stable=0;
        if(s->config.seek_timeout_ms && (uint32_t)(now-s->began)>=s->config.seek_timeout_ms){
            HomeSequence_Fail(s,HOME_SEEK_TIMEOUT);return;
        }
        for(uint8_t a=0;a<3;a++){
            uint8_t bit=(uint8_t)(1U<<a);
            if(s->parallel_phase[a]==HOME_SEEK && (low&bit)){
                /* Adopt的旧语义会关总EN，此处只继承屏蔽请求，随后保持使能。 */
                Adopt(s,a,now);s->enable=true;
                s->parallel_phase[a]=HOME_CONTACT;s->parallel_began[a]=now;
                s->parallel_low_seen&=(uint8_t)~bit;s->parallel_high_seen&=(uint8_t)~bit;
            }
            if(s->parallel_phase[a]==HOME_SEEK){
                if((uint32_t)(now-s->parallel_last_step[a])>=s->config.period_ms){
                    s->parallel_last_step[a]=now;s->seek[a]++;s->step_mask|=bit;
                }
                continue;
            }
            if(low&bit){
                s->parallel_high_seen&=(uint8_t)~bit;
                if(!(s->parallel_low_seen&bit)){
                    s->parallel_low_seen|=bit;s->parallel_low_since[a]=now;
                }
                if((uint32_t)(now-s->parallel_low_since[a])>=STABLE_MS){
                    s->contacts|=bit;stable|=bit;
                    /* 已确认且恢复稳定后结束本轴异常观察期限。 */
                    s->parallel_began[a]=now;
                }
            }else{
                s->parallel_low_seen&=(uint8_t)~bit;
                if(!(s->parallel_high_seen&bit)){
                    s->parallel_high_seen|=bit;s->parallel_high_since[a]=now;
                }
                if((uint32_t)(now-s->parallel_high_since[a])>=STABLE_MS){
                    if(s->contacts&bit){HomeSequence_Fail(s,HOME_UNEXPECTED);return;}
                    s->parallel_phase[a]=HOME_SEEK;s->parallel_last_step[a]=now;
                    s->parallel_high_seen&=(uint8_t)~bit;
                    if(s->contact_cancel_count<UINT32_MAX)s->contact_cancel_count++;
                    continue;
                }
            }
            if(!(stable&bit) && (uint32_t)(now-s->parallel_began[a])>=CONFIRM_MS){
                HomeSequence_Fail(s,HOME_UNSTABLE);return;
            }
        }
        if(stable==7U){
            /* 全到齐后在同一拍切方向，至少再等完整退回周期才发反向脉冲。 */
            s->step_mask=0;s->phase=HOME_BACKOFF;s->phase_began=now;
            s->dir_mask=(uint8_t)(~s->config.seek_dir&7U);s->back_pause_ms=0;
            s->parallel_high_seen=0;s->parallel_low_seen=0;
            for(uint8_t a=0;a<3;a++){
                s->parallel_phase[a]=HOME_BACKOFF;s->parallel_last_step[a]=now;
            }
        }
    }else if(s->phase==HOME_BACKOFF){
        if(HoldTick(s,now,low))return;
        for(uint8_t a=0;a<3;a++){
            uint8_t bit=(uint8_t)(1U<<a);
            if(!(s->rearmed&bit)){
                if(low&bit)s->parallel_high_seen&=(uint8_t)~bit;
                else{
                    if(!(s->parallel_high_seen&bit)){
                        s->parallel_high_seen|=bit;s->parallel_high_since[a]=now;
                    }
                    if((uint32_t)(now-s->parallel_high_since[a])>=STABLE_MS)s->rearm_request|=bit;
                }
            }
            if(s->completed&bit)continue;
            if((uint32_t)(now-s->phase_began)>=s->config.backoff[a]*s->config.backoff_period_ms+500U+s->back_pause_ms){
                HomeSequence_Fail(s,HOME_BACK_TIMEOUT);return;
            }
            if(s->back[a]>=s->config.backoff[a]){
                s->parallel_phase[a]=HOME_RELEASE;
                if(s->rearmed&bit)s->completed|=bit;
            }else if((uint32_t)(now-s->parallel_last_step[a])>=s->config.backoff_period_ms){
                s->parallel_last_step[a]=now;s->back[a]++;s->step_mask|=bit;
            }
        }
        if(s->completed==7U){s->phase=HOME_DONE;s->enable=false;s->step_mask=0;}
    }
    s->high=s->step_mask!=0;
}
