/** @file robot_home_pose.c @brief Post-homing pose and joint-limit model. */
#include "robot_home_pose.h"
#include "robot_config.h"

const RobotHomePose_t g_robot_home_pose = {
    .x_mm = HOMEPOSE_X_MM,
    .y_mm = HOMEPOSE_Y_MM,
    .z_mm = HOMEPOSE_Z_MM,
    .joint_deg = {HOMEPOSE_ROT_DEG, HOMEPOSE_LOW_DEG, HOMEPOSE_HIGH_DEG}
};

float RobotHomePose_MotorDegToJointDeg(RobotHomeAxis_t axis, float motor_deg)
{
    static const float direction[ROBOT_HOME_AXIS_COUNT] = {1.0f, -1.0f, -1.0f};
    if (axis >= ROBOT_HOME_AXIS_COUNT) return 0.0f;
    return g_robot_home_pose.joint_deg[axis] + direction[axis] * motor_deg / GEAR_RATIO;
}

bool RobotHomePose_IsJointAngleSafe(RobotHomeAxis_t axis, float joint_deg)
{
    const float tolerance = ACTUAL_JOINT_LIMIT_TOLERANCE_DEG;
    switch (axis) {
    case ROBOT_HOME_AXIS_ROT:
        return joint_deg >= ROT_MIN_DEG - tolerance && joint_deg <= ROT_MAX_DEG + tolerance;
    case ROBOT_HOME_AXIS_LOW:
        return joint_deg >= LOW_MIN_DEG - tolerance && joint_deg <= LOW_MAX_DEG + tolerance;
    case ROBOT_HOME_AXIS_HIGH:
        return joint_deg >= HIGH_MIN_DEG - tolerance && joint_deg <= HIGH_MAX_DEG + tolerance;
    default:
        return false;
    }
}
