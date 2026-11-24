/**
 * @file svc_homing.c
 * @brief 回零服务：寻找限位、逐轴停步、退回与失败处理的非阻塞状态机。
 */
#include "service/svc_homing.h"

#include "device/dev_joint.h"
#include "device/dev_limit_switch.h"
#include "platform_time.h"
#include "platform_critical.h"
#include "limit_irq_guard.h"
#include "robot_config.h"

#include <stdio.h>

#if ROBOTARM_SEQUENTIAL_HOMING
#include "service/svc_homing_sequence.h"
bool Svc_Homing_Start(void){return Svc_HomingSequence_Start();}
void Svc_Homing_Step(void){Svc_HomingSequence_MainHeartbeat();}
void Svc_Homing_Cancel(void){Svc_HomingSequence_Cancel();}
SvcHomingState_t Svc_Homing_GetState(void)
{
    SvcHomingSequenceStatus s;Svc_HomingSequence_GetStatus(&s);
    switch(s.flow.phase){
    case HOME_BOOT:return SVC_HOMING_IDLE;
    case HOME_BACKOFF:case HOME_RELEASE:return SVC_HOMING_BACKOFF;
    case HOME_DONE:return SVC_HOMING_COMPLETE;
    case HOME_FAILED:return SVC_HOMING_FAILED;
    default:return SVC_HOMING_SEEK;
    }
}
bool Svc_Homing_IsFinished(void)
{SvcHomingState_t s=Svc_Homing_GetState();return s==SVC_HOMING_COMPLETE || s==SVC_HOMING_FAILED;}
#else

static SvcHomingState_t s_state = SVC_HOMING_IDLE;
static bool s_hit[DEV_JOINT_COUNT];
static uint32_t s_step_count;
static uint32_t s_backoff_step;
static uint32_t s_last_step_us;
static int32_t s_backoff[DEV_JOINT_COUNT];
static const bool s_seek_dir[DEV_JOINT_COUNT]={
    HOMING_SEEK_M1_DIR_HIGH,HOMING_SEEK_M2_DIR_HIGH,HOMING_SEEK_M3_DIR_HIGH
};

/** 启动寻限位；重复启动活动回零返回 false，不在函数内等待到位。 */
bool Svc_Homing_Start(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    if (s_state == SVC_HOMING_SEEK || s_state == SVC_HOMING_BACKOFF ||
        LimitIrqGuard_GetLatchMask()!=0U) {
        PlatformCritical_Exit(key);return false;
    }

    s_hit[DEV_JOINT_M1] = false;
    s_hit[DEV_JOINT_M2] = false;
    s_hit[DEV_JOINT_M3] = false;
    s_step_count = 0U;
    s_backoff_step = 0U;
    s_last_step_us = PlatformTime_NowUs();
    s_backoff[DEV_JOINT_M1] = (int32_t)(HOMING_BACKOFF_M1_DEG * UNITS_PER_DEGREE);
    s_backoff[DEV_JOINT_M2] = (int32_t)(HOMING_BACKOFF_M2_DEG * UNITS_PER_DEGREE);
    s_backoff[DEV_JOINT_M3] = (int32_t)(HOMING_BACKOFF_M3_DEG * UNITS_PER_DEGREE);

    for (uint8_t joint=0; joint<DEV_JOINT_COUNT; ++joint)
        Dev_Joint_SetDirectionLevel((DevJointId_t)joint,s_seek_dir[joint]);
    s_state = SVC_HOMING_SEEK;
    PlatformCritical_Exit(key);
    printf("# [Homing] Starting.\r\n");
    return true;
}

typedef enum { HOME_LOG_NONE,HOME_LOG_TIMEOUT,HOME_LOG_BACKOFF,HOME_LOG_STUCK,HOME_LOG_COMPLETE } HomingLog_t;

/** 调用者已进入短临界区：只更新状态、读取GPIO和STEP；返回日志事件供区外输出。 */
static HomingLog_t StepLocked(void)
{
    if (LimitIrqGuard_GetLatchMask()!=0U) { s_state=SVC_HOMING_FAILED;return HOME_LOG_NONE; }
    if (s_state != SVC_HOMING_SEEK && s_state != SVC_HOMING_BACKOFF) return HOME_LOG_NONE;
    uint32_t now_us = PlatformTime_NowUs();
    if ((uint32_t)(now_us - s_last_step_us) < HOMING_STEP_DELAY_US) return HOME_LOG_NONE;
    s_last_step_us = now_us;

    if (s_state == SVC_HOMING_SEEK) {
        if (++s_step_count > HOMING_MAX_STEPS) {
            s_state = SVC_HOMING_FAILED;
            return HOME_LOG_TIMEOUT;
        }
        for (uint8_t joint = DEV_JOINT_M1; joint < DEV_JOINT_COUNT; joint++) {
            if (!s_hit[joint]) {
                s_hit[joint] = Dev_LimitSwitch_IsTriggered((DevJointId_t)joint);
                /* 先读取限位，避免已经压住开关的轴再次朝限位推进。 */
                if (!s_hit[joint] && !Dev_Joint_Step((DevJointId_t)joint)) {
                    s_state = SVC_HOMING_FAILED;
                    return HOME_LOG_NONE;
                }
            }
        }
        if (s_hit[0] && s_hit[1] && s_hit[2]) {
            for (uint8_t joint=0; joint<DEV_JOINT_COUNT; ++joint)
                Dev_Joint_SetDirectionLevel((DevJointId_t)joint,!s_seek_dir[joint]);
            s_state = SVC_HOMING_BACKOFF;
            return HOME_LOG_BACKOFF;
        }
        return HOME_LOG_NONE;
    }

    bool any_step = false;
    for (uint8_t joint = DEV_JOINT_M1; joint < DEV_JOINT_COUNT; joint++) {
        if ((int32_t)s_backoff_step < s_backoff[joint]) {
            if (!Dev_Joint_Step((DevJointId_t)joint)) {
                s_state = SVC_HOMING_FAILED;
                return HOME_LOG_NONE;
            }
            any_step = true;
        }
    }
    s_backoff_step++;
    if (!any_step) {
        /* 退回完成仍压住限位，说明方向、接线或行程需要检查，不能宣布成功。 */
        for (uint8_t joint=0; joint<DEV_JOINT_COUNT; ++joint) {
            if (Dev_LimitSwitch_IsTriggered((DevJointId_t)joint)) {
                s_state = SVC_HOMING_FAILED;
                return HOME_LOG_STUCK;
            }
        }
        s_state = SVC_HOMING_COMPLETE;
        return HOME_LOG_COMPLETE;
    }
    return HOME_LOG_NONE;
}

/** 每轮最多给每个轴一个STEP；临界区内固定工作量，日志在恢复中断后输出。 */
void Svc_Homing_Step(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    HomingLog_t event=StepLocked();
    bool hit[3]={s_hit[0],s_hit[1],s_hit[2]};
    PlatformCritical_Exit(key);
    switch(event) {
    case HOME_LOG_TIMEOUT:
        printf("# [Homing] timeout M1=%d M2=%d M3=%d\r\n",hit[0],hit[1],hit[2]);break;
    case HOME_LOG_BACKOFF:
        printf("# [Homing] All switches hit. Backing off M1=%.0f M2=%.0f M3=%.0f deg...\r\n",
               HOMING_BACKOFF_M1_DEG,HOMING_BACKOFF_M2_DEG,HOMING_BACKOFF_M3_DEG);break;
    case HOME_LOG_STUCK: printf("# [Homing] switch did not release.\r\n");break;
    case HOME_LOG_COMPLETE: printf("# [Homing] Complete.\r\n");break;
    default: break;
    }
}

SvcHomingState_t Svc_Homing_GetState(void) { return s_state; }
void Svc_Homing_Cancel(void) { s_state = SVC_HOMING_FAILED; }
bool Svc_Homing_IsFinished(void) { return s_state == SVC_HOMING_COMPLETE || s_state == SVC_HOMING_FAILED; }
#endif
