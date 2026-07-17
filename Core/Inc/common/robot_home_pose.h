/** @file robot_home_pose.h @brief Post-homing pose and joint-limit model. */
#ifndef ROBOTARM_ROBOT_HOME_POSE_H
#define ROBOTARM_ROBOT_HOME_POSE_H

#include <stdbool.h>

typedef enum {
    ROBOT_HOME_AXIS_ROT = 0,
    ROBOT_HOME_AXIS_LOW,
    ROBOT_HOME_AXIS_HIGH,
    ROBOT_HOME_AXIS_COUNT
} RobotHomeAxis_t;

typedef struct {
    float x_mm;
    float y_mm;
    float z_mm;
    float joint_deg[ROBOT_HOME_AXIS_COUNT];
} RobotHomePose_t;

extern const RobotHomePose_t g_robot_home_pose;

float RobotHomePose_MotorDegToJointDeg(RobotHomeAxis_t axis, float motor_deg);
bool RobotHomePose_IsJointAngleSafe(RobotHomeAxis_t axis, float joint_deg);

#endif /* ROBOTARM_ROBOT_HOME_POSE_H */
