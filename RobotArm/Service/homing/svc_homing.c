#include "service/svc_homing.h"

#include "device/dev_joint.h"
#include "device/dev_limit_switch.h"
#include "platform_time.h"
#include "robot_config.h"

#include <stdio.h>

static SvcHomingState_t s_state = SVC_HOMING_IDLE;
static bool s_hit[DEV_JOINT_COUNT];
static uint32_t s_step_count;
static uint32_t s_backoff_step;
static uint32_t s_last_step_us;
static int32_t s_backoff[DEV_JOINT_COUNT];

bool Svc_Homing_Start(void)
{
    if (s_state == SVC_HOMING_SEEK || s_state == SVC_HOMING_BACKOFF) return false;

    s_hit[DEV_JOINT_M1] = false;
    s_hit[DEV_JOINT_M2] = false;
    s_hit[DEV_JOINT_M3] = false;
    s_step_count = 0U;
    s_backoff_step = 0U;
    s_last_step_us = PlatformTime_NowUs();
    s_backoff[DEV_JOINT_M1] = (int32_t)(HOMING_BACKOFF_M1_DEG * UNITS_PER_DEGREE);
    s_backoff[DEV_JOINT_M2] = (int32_t)(HOMING_BACKOFF_M2_DEG * UNITS_PER_DEGREE);
    s_backoff[DEV_JOINT_M3] = (int32_t)(HOMING_BACKOFF_M3_DEG * UNITS_PER_DEGREE);

    Dev_Joint_SetDirection(DEV_JOINT_M1, true);
    Dev_Joint_SetDirection(DEV_JOINT_M2, true);
    Dev_Joint_SetDirection(DEV_JOINT_M3, false);
    s_state = SVC_HOMING_SEEK;
    printf("# [Homing] Starting.\r\n");
    return true;
}

void Svc_Homing_Step(void)
{
    if (s_state != SVC_HOMING_SEEK && s_state != SVC_HOMING_BACKOFF) return;
    uint32_t now_us = PlatformTime_NowUs();
    if ((uint32_t)(now_us - s_last_step_us) < HOMING_STEP_DELAY_US) return;
    s_last_step_us = now_us;

    if (s_state == SVC_HOMING_SEEK) {
        if (++s_step_count > HOMING_MAX_STEPS) {
            s_state = SVC_HOMING_FAILED;
            printf("# [Homing] timeout M1=%d M2=%d M3=%d\r\n", s_hit[0], s_hit[1], s_hit[2]);
            return;
        }
        for (uint8_t joint = DEV_JOINT_M1; joint < DEV_JOINT_COUNT; joint++) {
            if (!s_hit[joint]) {
                Dev_Joint_Step((DevJointId_t)joint);
                s_hit[joint] = Dev_LimitSwitch_IsTriggered((DevJointId_t)joint);
            }
        }
        if (s_hit[0] && s_hit[1] && s_hit[2]) {
            Dev_Joint_SetDirection(DEV_JOINT_M1, false);
            Dev_Joint_SetDirection(DEV_JOINT_M2, false);
            Dev_Joint_SetDirection(DEV_JOINT_M3, true);
            s_state = SVC_HOMING_BACKOFF;
            printf("# [Homing] All switches hit. Backing off M1=%.0f M2=%.0f M3=%.0f deg...\r\n",
                   HOMING_BACKOFF_M1_DEG, HOMING_BACKOFF_M2_DEG, HOMING_BACKOFF_M3_DEG);
        }
        return;
    }

    bool any_step = false;
    for (uint8_t joint = DEV_JOINT_M1; joint < DEV_JOINT_COUNT; joint++) {
        if ((int32_t)s_backoff_step < s_backoff[joint]) {
            Dev_Joint_Step((DevJointId_t)joint);
            any_step = true;
        }
    }
    s_backoff_step++;
    if (!any_step) {
        s_state = SVC_HOMING_COMPLETE;
        printf("# [Homing] Complete.\r\n");
    }
}

SvcHomingState_t Svc_Homing_GetState(void) { return s_state; }
bool Svc_Homing_IsFinished(void) { return s_state == SVC_HOMING_COMPLETE || s_state == SVC_HOMING_FAILED; }
