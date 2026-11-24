/**
 * @file waveform_service.c
 * @brief 有限波形状态机：方向建立、定宽高电平、低电平等待与停止收尾。
 */
#include "waveform_service.h"
#include "device/dev_joint.h"
#include "device/dev_limit_switch.h"
#include "platform_critical.h"
#include "limit_irq_guard.h"
#include <stddef.h>

/* TIM6 配置来源 Core/Src/tim.c：84 MHz / 84 / 20 = 50 kHz。
 * 高电平 5 tick=100 us，方向建立 100 tick=2 ms；实际值须用仪器验收。
 * 频率均整除 50000，避免向用户隐瞒量化误差。最大任务约 10.002 s。
 */
static volatile WaveformStatus_t s_status;
static volatile uint16_t s_heartbeat;
static uint16_t s_countdown, s_period, s_target;
static bool s_high, s_healthy;

static uint8_t ReadLimitMask(void)
{
    uint8_t mask=0U;
    for (unsigned i=0; i<DEV_JOINT_COUNT; ++i)
        if (Dev_LimitSwitch_IsTriggered((DevJointId_t)i)) mask|=(uint8_t)(1U<<i);
    return mask;
}
/* 调用方已处于 ISR 或短临界区；无论停止原因如何，先低 STEP 再结束任务。 */
static void StopLocked(WaveformState_t reason)
{
    for (unsigned i=0; i<DEV_JOINT_COUNT; ++i)
        (void)Dev_Joint_WriteStepLevel((DevJointId_t)i, false);
    Dev_Joint_EnableAll(false);
    s_high=false;
    s_status.state=reason;
}
void WaveformService_Init(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    s_status=(WaveformStatus_t){.state=WAVE_IDLE,.next_id=1U};
    s_healthy=false; s_heartbeat=0;
    StopLocked(WAVE_IDLE);
    PlatformCritical_Exit(key);
}
void WaveformService_Feed(bool healthy)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    s_heartbeat=0; s_healthy=healthy;
    if (!healthy && s_status.state==WAVE_RUNNING) StopLocked(WAVE_ENCODER);
    PlatformCritical_Exit(key);
}
bool WaveformService_Start(uint32_t id, uint8_t axis, bool dir_high,
                           uint16_t pulses, uint16_t hz)
{
    if (axis>=DEV_JOINT_COUNT || pulses==0 || pulses>100 ||
        !(hz==10 || hz==20 || hz==25 || hz==50 || hz==100)) return false;
    PlatformCriticalState_t key=PlatformCritical_Enter();
    bool accepted=false;
    /* next_id 不回绕；UINT32_MAX 到达后需人工复位新会话。 */
    if (LimitIrqGuard_GetLatchMask()==0U && s_status.state!=WAVE_RUNNING && s_healthy && s_heartbeat<25000U &&
        id==s_status.next_id && id!=UINT32_MAX && ReadLimitMask()==0U) {
        StopLocked(WAVE_IDLE);
        s_status.axis=axis; s_status.emitted=0;
        s_status.limit_mask=s_status.limit_raw_mask=s_status.limit_source=0U;
        if (Dev_Joint_SetDirection((DevJointId_t)axis,!dir_high)) {
            s_status.next_id++; s_period=(uint16_t)(50000U/hz);
            s_target=pulses; s_countdown=100U;
            s_status.state=WAVE_RUNNING; accepted=true;
        } else StopLocked(WAVE_OUTPUT);
    }
    PlatformCritical_Exit(key);
    return accepted;
}
void WaveformService_Stop(WaveformState_t reason)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    if (LimitIrqGuard_GetLatchMask()!=0U) reason=WAVE_IRQ_OVERLOAD;
    StopLocked(reason);
    PlatformCritical_Exit(key);
}
void WaveformService_GetStatus(WaveformStatus_t *out)
{
    if (out==NULL) return;
    PlatformCriticalState_t key=PlatformCritical_Enter();
    *out=s_status;
    PlatformCritical_Exit(key);
}
void WaveformService_OnLimitFromISR(uint8_t axis)
{
    if (axis>=DEV_JOINT_COUNT) return;
    PlatformCriticalState_t key=PlatformCritical_Enter();
    if (s_status.limit_edges[axis]!=UINT32_MAX) ++s_status.limit_edges[axis];
    if (s_status.state==WAVE_RUNNING) {
        s_status.limit_mask=(uint8_t)(1U<<axis);
        s_status.limit_raw_mask=ReadLimitMask();
        s_status.limit_source=1U;
        StopLocked(WAVE_LIMIT);
    }
    PlatformCritical_Exit(key);
}
static void OnTickLocked(void)
{
    if (LimitIrqGuard_GetLatchMask()!=0U) { StopLocked(WAVE_IRQ_OVERLOAD);return; }
    if (s_heartbeat<25000U) ++s_heartbeat;
    if (s_status.state!=WAVE_RUNNING) return;
    uint8_t mask=ReadLimitMask();
    if (mask!=0U) {
        s_status.limit_mask=mask; s_status.limit_raw_mask=mask; s_status.limit_source=2U;
        StopLocked(WAVE_LIMIT); return;
    }
    if (s_heartbeat>=25000U) { StopLocked(WAVE_HEARTBEAT); return; }
    if (--s_countdown!=0U) return;
    if (s_high) {
        if (!Dev_Joint_WriteStepLevel((DevJointId_t)s_status.axis,false)) {
            StopLocked(WAVE_OUTPUT); return;
        }
        s_high=false;
        if (s_status.emitted>=s_target) StopLocked(WAVE_DONE);
        else s_countdown=(uint16_t)(s_period-5U);
    } else {
        if (!Dev_Joint_WriteStepLevel((DevJointId_t)s_status.axis,true)) {
            StopLocked(WAVE_OUTPUT); return;
        }
        ++s_status.emitted; s_high=true; s_countdown=5U;
    }
}
void WaveformService_OnTickFromISR(void)
{
    /* 防止更高优先级 EXTI 在收尾后被旧 TIM6 调用继续拉高 STEP。 */
    PlatformCriticalState_t key=PlatformCritical_Enter();
    OnTickLocked();
    PlatformCritical_Exit(key);
}
