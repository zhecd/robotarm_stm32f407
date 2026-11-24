/**
 * @file ctrl_motion_engine.c
 * @brief 运动内部实现：单生产者/单消费者帧队列与定时整数插补。
 * @details 主循环生产帧，定时 ISR 消费；故障锁存和限位消抖暂停均在输出 STEP 前检查。
 */

#include "ctrl_motion_engine.h"
#include "device/dev_joint.h"
#include "device/dev_limit_switch.h"
#include "platform_critical.h"
#include "platform_time.h"
#include "robot_config.h"
#include <stdio.h>
#include <stdlib.h>

static MotionFrame_t s_buffer[RING_BUFFER_SIZE];
static volatile uint16_t s_head = 0;
static volatile uint16_t s_tail = 0;

static volatile bool s_running      = false;
static volatile bool s_faulted      = false;
static volatile bool s_limit_monitoring_enabled = false;
static volatile uint8_t s_pending_limit_mask = 0U;
/* 每轴独立保存最后边沿与版本。不同轴不能相互重置10ms消抖窗口。 */
static volatile uint32_t s_limit_event_ms[3];
static volatile uint32_t s_limit_generation[3];
static volatile MotionFaultReason_t s_fault_reason = MOTION_FAULT_NONE;
static MotionFrame_t s_cur_frame;
static uint32_t      s_cur_tick     = 0;
static uint64_t      s_acc_m1       = 0;
static uint64_t      s_acc_m2       = 0;
static uint64_t      s_acc_m3       = 0;
static uint32_t      s_abs_m1       = 0;
static uint32_t      s_abs_m2       = 0;
static uint32_t      s_abs_m3       = 0;

static int32_t s_theory_m1 = 0;
static int32_t s_theory_m2 = 0;
static int32_t s_theory_m3 = 0;

void Ctrl_MotionEngine_Init(void)
{
    s_head = 0;
    s_tail = 0;
    s_running  = false;
    s_faulted = false;
    s_limit_monitoring_enabled = false;
    s_pending_limit_mask = 0U;
    for (unsigned axis=0;axis<3;axis++) {
        s_limit_event_ms[axis]=0U;
        s_limit_generation[axis]=0U;
    }
    s_fault_reason = MOTION_FAULT_NONE;
    s_cur_tick = 0;
    s_theory_m1 = 0;
    s_theory_m2 = 0;
    s_theory_m3 = 0;
}

bool Ctrl_MotionEngine_PushFrame(const MotionFrame_t *frame)
{
    /* 每个 tick 每轴最多输出一个 STEP。零时长或步数超过节拍数都会丢步，
       因此在入队前拒绝；零步数但有时长的帧则保留，用于低速轨迹计时。 */
    if (!frame || frame->total_ticks == 0U) return false;
    uint32_t a = (uint32_t)llabs((long long)frame->delta_m1);
    uint32_t b = (uint32_t)llabs((long long)frame->delta_m2);
    uint32_t c = (uint32_t)llabs((long long)frame->delta_m3);
    if (a > frame->total_ticks || b > frame->total_ticks || c > frame->total_ticks) return false;
    PlatformCriticalState_t state = PlatformCritical_Enter();
    uint16_t next = (s_head + 1) % RING_BUFFER_SIZE;
    if (s_faulted || next == s_tail) {
        PlatformCritical_Exit(state);
        return false;
    }
    s_buffer[s_head] = *frame;
    s_head = next;
    PlatformCritical_Exit(state);
    return true;
}

static bool PopFrame(MotionFrame_t *out)
{
    if (!out || s_head == s_tail) return false;
    *out = s_buffer[s_tail];
    s_tail = (s_tail + 1) % RING_BUFFER_SIZE;
    return true;
}

void Ctrl_MotionEngine_Clear(void)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_tail = s_head;
    PlatformCritical_Exit(state);
}

void Ctrl_MotionEngine_EmergencyStop(void)
{
    Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_NONE);
}

void Ctrl_MotionEngine_EmergencyStopWithReason(MotionFaultReason_t reason)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_head = 0;
    s_tail = 0;
    s_running = false;
    s_cur_tick = 0;
    s_acc_m1 = s_acc_m2 = s_acc_m3 = 0;
    /* 保留首个故障，防止后续派生错误覆盖真正的触发原因。 */
    if (!s_faulted) s_fault_reason = reason;
    s_faulted = true;
    PlatformCritical_Exit(state);
}

void Ctrl_MotionEngine_EnableLimitMonitoring(bool enabled)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_limit_monitoring_enabled = enabled;
    s_pending_limit_mask = 0U;
    PlatformCritical_Exit(state);
}

void Ctrl_MotionEngine_NotifyLimitSwitch(uint8_t joint_mask)
{
    if (!s_limit_monitoring_enabled) return;
    joint_mask &= (MOTION_LIMIT_M1 | MOTION_LIMIT_M2 | MOTION_LIMIT_M3);
    if (joint_mask == 0U) return;
    PlatformCriticalState_t state=PlatformCritical_Enter();
    s_pending_limit_mask |= joint_mask;
    uint32_t now=PlatformTime_NowMs();
    for (unsigned axis=0;axis<3;axis++) {
        if ((joint_mask&(1U<<axis))==0U) continue;
        s_limit_event_ms[axis]=now;
        ++s_limit_generation[axis];
    }
    PlatformCritical_Exit(state);
}

void Ctrl_MotionEngine_ServiceSafety(void)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    uint8_t pending = s_pending_limit_mask;
    uint32_t event_ms[3],generation[3];
    for (unsigned axis=0;axis<3;axis++) {
        event_ms[axis]=s_limit_event_ms[axis];
        generation[axis]=s_limit_generation[axis];
    }
    PlatformCritical_Exit(state);

    if (pending==0U) return;
    uint32_t now=PlatformTime_NowMs();
    uint8_t qualified=0U;
    bool active=false;
    for (unsigned axis=0;axis<3;axis++) {
        uint8_t bit=(uint8_t)(1U<<axis);
        if ((pending&bit)==0U || (uint32_t)(now-event_ms[axis])<10U) continue;
        qualified|=bit;
        if (Dev_LimitSwitch_IsTriggered((DevJointId_t)axis)) active=true;
    }
    if (qualified==0U) return;
    /* 先锁存停止，再清除暂停标志，防止两者之间的 ISR 多输出一次脉冲。 */
    if (active) Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_LIMIT_SWITCH);
    /* GPIO 读取不占用临界区。若期间来了新边沿，保留新事件供下一次消抖。 */
    state = PlatformCritical_Enter();
    for (unsigned axis=0;axis<3;axis++) {
        uint8_t bit=(uint8_t)(1U<<axis);
        if ((qualified&bit)!=0U && generation[axis]==s_limit_generation[axis])
            s_pending_limit_mask&=(uint8_t)~bit;
    }
    PlatformCritical_Exit(state);
}

bool Ctrl_MotionEngine_HasFault(void)
{
    return s_faulted;
}

MotionFaultReason_t Ctrl_MotionEngine_GetFaultReason(void)
{
    return s_fault_reason;
}

void Ctrl_MotionEngine_ClearFault(void)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_faulted = false;
    s_fault_reason = MOTION_FAULT_NONE;
    s_pending_limit_mask = 0U;
    PlatformCritical_Exit(state);
}

bool Ctrl_MotionEngine_IsRunning(void)
{
    return s_running;
}

uint16_t Ctrl_MotionEngine_GetQueueCount(void)
{
    PlatformCriticalState_t state=PlatformCritical_Enter();
    uint16_t head=s_head, tail=s_tail;
    PlatformCritical_Exit(state);
    return head>=tail ? (uint16_t)(head-tail) : (uint16_t)(RING_BUFFER_SIZE-tail+head);
}

void Ctrl_MotionEngine_GetTheorySteps(int32_t *m1, int32_t *m2, int32_t *m3)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    if (m1) *m1 = s_theory_m1;
    if (m2) *m2 = s_theory_m2;
    if (m3) *m3 = s_theory_m3;
    PlatformCritical_Exit(state);
}

void Ctrl_MotionEngine_ResetTheorySteps(void)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_theory_m1 = 0;
    s_theory_m2 = 0;
    s_theory_m3 = 0;
    PlatformCritical_Exit(state);
}

void Ctrl_MotionEngine_AdjustTheorySteps(int32_t dm1, int32_t dm2, int32_t dm3)
{
    PlatformCriticalState_t state = PlatformCritical_Enter();
    s_theory_m1 += dm1;
    s_theory_m2 += dm2;
    s_theory_m3 += dm3;
    PlatformCritical_Exit(state);
}

/* TIM6 ISR: Bresenham step generation */

/** 每次调用执行一个定时节拍；时间上限与 STEP 脉宽需后续实机测量。 */
void Ctrl_MotionEngine_OnStepTickFromISR(void)
{
    /* 限位边沿到来后先暂停脉冲，主循环完成消抖后决定锁存或恢复。
       ISR 内只做固定工作量的整数插补，不读 I2C、不打印、不等待。 */
    if (s_faulted || s_pending_limit_mask != 0U) return;

    if (!s_running) {
        if (!PopFrame(&s_cur_frame)) return;
        s_running  = true;
        s_cur_tick = 0;

        Dev_Joint_SetDirection(DEV_JOINT_M1, s_cur_frame.delta_m1 >= 0);
        Dev_Joint_SetDirection(DEV_JOINT_M2, s_cur_frame.delta_m2 >= 0);
        Dev_Joint_SetDirection(DEV_JOINT_M3, s_cur_frame.delta_m3 >= 0);

        s_abs_m1 = (uint32_t)llabs((long long)s_cur_frame.delta_m1);
        s_abs_m2 = (uint32_t)llabs((long long)s_cur_frame.delta_m2);
        s_abs_m3 = (uint32_t)llabs((long long)s_cur_frame.delta_m3);

        s_acc_m1 = s_cur_frame.total_ticks / 2;
        s_acc_m2 = s_cur_frame.total_ticks / 2;
        s_acc_m3 = s_cur_frame.total_ticks / 2;
    }

    s_acc_m1 += s_abs_m1;
    if (s_acc_m1 >= s_cur_frame.total_ticks) {
        Dev_Joint_Step(DEV_JOINT_M1);
        s_acc_m1 -= s_cur_frame.total_ticks;
    }

    s_acc_m2 += s_abs_m2;
    if (s_acc_m2 >= s_cur_frame.total_ticks) {
        Dev_Joint_Step(DEV_JOINT_M2);
        s_acc_m2 -= s_cur_frame.total_ticks;
    }

    s_acc_m3 += s_abs_m3;
    if (s_acc_m3 >= s_cur_frame.total_ticks) {
        Dev_Joint_Step(DEV_JOINT_M3);
        s_acc_m3 -= s_cur_frame.total_ticks;
    }

    if (++s_cur_tick >= s_cur_frame.total_ticks)
        s_running = false;
}
