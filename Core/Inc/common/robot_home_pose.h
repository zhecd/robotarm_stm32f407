/**
 * @file    home_pose.h
 * @brief   Single source of truth for the post-homing robot pose.
 */

#ifndef __HOME_POSE_H__
#define __HOME_POSE_H__

#include <stdbool.h>

typedef enum {
    HOME_AXIS_ROT = 0,
    HOME_AXIS_LOW,
    HOME_AXIS_HIGH,
    HOME_AXIS_COUNT
} HomeAxis_t;

typedef struct {
    float x_mm;
    float y_mm;
    float z_mm;
    float joint_deg[HOME_AXIS_COUNT];
} HomePose_t;

extern const HomePose_t g_home_pose;

/* Convert the continuous motor-side AS5600 angle to a kinematic joint angle. */
float HomePose_EncoderMotorDegToJointDeg(HomeAxis_t axis, float motor_deg);

/* Tests against configured joint limits, including the small encoder tolerance. */
bool HomePose_IsJointAngleSafe(HomeAxis_t axis, float joint_deg);

#endif /* __HOME_POSE_H__ */
