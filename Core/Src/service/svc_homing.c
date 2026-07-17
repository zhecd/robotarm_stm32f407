/** @file svc_homing.c @brief Limit-switch-based joint homing service. */
#include "service/svc_homing.h"

#include "device/dev_joint.h"
#include "device/dev_limit_switch.h"
#include "os/os_adapter.h"
#include "robot_config.h"
#include <stdio.h>

bool Svc_Homing_Execute(void)
{
    printf("[Homing] Starting...\r\n");

    Dev_Joint_SetDirection(DEV_JOINT_M1, true);
    Dev_Joint_SetDirection(DEV_JOINT_M2, true);
    Dev_Joint_SetDirection(DEV_JOINT_M3, false);

    bool m1_done = false, m2_done = false, m3_done = false;
    uint32_t step = 0U;
    while (!m1_done || !m2_done || !m3_done) {
        if (++step > HOMING_MAX_STEPS) {
            printf("[Homing] timeout M1=%d M2=%d M3=%d\r\n", m1_done, m2_done, m3_done);
            return false;
        }
        if (!m1_done) {
            Dev_Joint_Step(DEV_JOINT_M1);
            m1_done = Dev_LimitSwitch_IsTriggered(DEV_JOINT_M1);
        }
        if (!m2_done) {
            Dev_Joint_Step(DEV_JOINT_M2);
            m2_done = Dev_LimitSwitch_IsTriggered(DEV_JOINT_M2);
        }
        if (!m3_done) {
            Dev_Joint_Step(DEV_JOINT_M3);
            m3_done = Dev_LimitSwitch_IsTriggered(DEV_JOINT_M3);
        }
        Os_DelayUs(HOMING_STEP_DELAY_US);
    }

    const int32_t backoff[DEV_JOINT_COUNT] = {
        (int32_t)(HOMING_BACKOFF_M1_DEG * UNITS_PER_DEGREE),
        (int32_t)(HOMING_BACKOFF_M2_DEG * UNITS_PER_DEGREE),
        (int32_t)(HOMING_BACKOFF_M3_DEG * UNITS_PER_DEGREE)
    };
    printf("[Homing] All switches hit. Backing off M1=%.0f M2=%.0f M3=%.0f deg...\r\n",
           HOMING_BACKOFF_M1_DEG, HOMING_BACKOFF_M2_DEG, HOMING_BACKOFF_M3_DEG);

    Dev_Joint_SetDirection(DEV_JOINT_M1, false);
    Dev_Joint_SetDirection(DEV_JOINT_M2, false);
    Dev_Joint_SetDirection(DEV_JOINT_M3, true);
    int32_t max_steps = backoff[DEV_JOINT_M1];
    for (uint8_t joint = DEV_JOINT_M2; joint < DEV_JOINT_COUNT; joint++)
        if (backoff[joint] > max_steps) max_steps = backoff[joint];

    for (int32_t i = 0; i < max_steps; i++) {
        for (uint8_t joint = DEV_JOINT_M1; joint < DEV_JOINT_COUNT; joint++)
            if (i < backoff[joint]) Dev_Joint_Step((DevJointId_t)joint);
        Os_DelayUs(HOMING_STEP_DELAY_US);
    }

    printf("[Homing] Complete.\r\n");
    return true;
}
