/** @file m1_reference.c @brief 无HAL、无全局状态的局部坐标生命周期。 */
#include "m1_reference.h"
#include <math.h>
#include <stddef.h>

static void Invalidate(M1Reference *r,M1ReferenceReason reason)
{
    if (!r || r->state==M1_REF_INVALID) return;
    r->state=M1_REF_INVALID;r->reason=reason;
    /* 原点保留供取证；相对角清零且VALID为假，调用方不能使用旧的有效值。 */
    r->relative_motor_deg=0.0f;r->relative_joint_deg=0.0f;
}

static bool HomeCompleted(const M1Test *t)
{
    return t && t->state==M1_DONE && t->reason==M1_OK && t->used &&
        !t->enable && !t->step && t->direction_confirmed && t->release_confirmed &&
        t->jog_steps==0U && t->seek_steps>0U && t->seek_steps<=M1_SEEK_STEPS &&
        t->backoff_steps==M1_BACKOFF_STEPS;
}

void M1Reference_Init(M1Reference *r,float ratio)
{
    if (!r) return;
    *r=(M1Reference){.state=M1_REF_UNSET,.reason=M1_REF_NOT_HOME,.gear_ratio=ratio};
    if (!isfinite(ratio) || ratio<=0.0f) Invalidate(r,M1_REF_BAD_CONFIG);
}

void M1Reference_ObserveState(M1Reference *r,const M1Test *t,uint32_t now)
{
    if (!r || r->state==M1_REF_INVALID) return;
    if (!t || t->state==M1_STOPPED || t->state==M1_FAILED) {
        Invalidate(r,M1_REF_MOTION_FAULT);return;
    }
    if (r->state==M1_REF_UNSET) {
        if (HomeCompleted(t)) {
            r->state=M1_REF_WAIT_SAMPLE;r->reason=M1_REF_WAIT_FEEDBACK;
            r->waiting_since_ms=now;
        }
        return; /* JOG的DONE不能成为HOME原点。 */
    }
    if (!HomeCompleted(t)) { Invalidate(r,M1_REF_MOTION_FAULT);return; }
    /* 所有时间差均按uint32_t相减，覆盖HAL毫秒计数回绕。 */
    if (r->state==M1_REF_WAIT_SAMPLE &&
        (uint32_t)(now-r->waiting_since_ms)>=M1_FEEDBACK_STALE_MS)
        Invalidate(r,M1_REF_SAMPLE_TIMEOUT);
    else if (r->state==M1_REF_VALID &&
        (uint32_t)(now-r->last_sample_started_ms)>=M1_FEEDBACK_STALE_MS)
        Invalidate(r,M1_REF_BAD_FEEDBACK);
}

void M1Reference_OnSample(M1Reference *r,const M1Test *t,const M1ReferenceSample *s)
{
    if (!r || r->state==M1_REF_INVALID || r->state==M1_REF_UNSET) return;
    if (!s) { Invalidate(r,M1_REF_BAD_FEEDBACK);return; }
    M1Reference_ObserveState(r,t,s->finished_ms);
    if (r->state==M1_REF_INVALID) return;
    if (!s->input_config_ok) { Invalidate(r,M1_REF_BAD_CONFIG);return; }
    if (s->limit_mask) { Invalidate(r,M1_REF_LIMIT_ACTIVE);return; }
    if (!s->valid || !isfinite(s->motor_deg) ||
        (uint32_t)(s->finished_ms-s->started_ms)>=M1_FEEDBACK_STALE_MS) {
        Invalidate(r,M1_REF_BAD_FEEDBACK);return;
    }
    if (r->state==M1_REF_WAIT_SAMPLE) {
        /* 符号位表示该读取在首次观察到DONE之前已开始；跨状态读取不能定原点。
         * 等待上限仅100ms，所有正常时差远小于uint32_t半周期。 */
        if ((uint32_t)(s->started_ms-r->waiting_since_ms)>UINT32_MAX/2U) return;
        r->origin_motor_deg=s->motor_deg;r->relative_motor_deg=0.0f;
        r->relative_joint_deg=0.0f;r->state=M1_REF_VALID;r->reason=M1_REF_OK;
    } else {
        /* 同一时间戳不能刷新有效期，较旧样本也不能覆盖新角度。 */
        uint32_t advance=s->started_ms-r->last_sample_started_ms;
        if (advance==0U || advance>UINT32_MAX/2U) return;
        float motor_delta=s->motor_deg-r->origin_motor_deg;
        float joint_delta=motor_delta/r->gear_ratio;
        if (!isfinite(motor_delta) || !isfinite(joint_delta)) {
            Invalidate(r,M1_REF_BAD_FEEDBACK);return;
        }
        r->relative_motor_deg=motor_delta;r->relative_joint_deg=joint_delta;
    }
    r->last_sample_started_ms=s->started_ms;
}
