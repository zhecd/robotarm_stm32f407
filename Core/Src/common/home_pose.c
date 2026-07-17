/**
 * @file    home_pose.c
 * @brief   Post-homing coordinate reference and encoder-to-joint conversion.
 */

#include "home_pose.h"
#include "robot_config.h"

const HomePose_t g_home_pose = {
    .x_mm = HOMEPOSE_X_MM,
    .y_mm = HOMEPOSE_Y_MM,
    .z_mm = HOMEPOSE_Z_MM,
    .joint_deg = {HOMEPOSE_ROT_DEG, HOMEPOSE_LOW_DEG, HOMEPOSE_HIGH_DEG}
};

float HomePose_EncoderMotorDegToJointDeg(HomeAxis_t axis, float motor_deg)
{
    if (axis >= HOME_AXIS_COUNT)
        return 0.0f;

    /* M1 follows the kinematic positive direction.  M2/M3 are inverted by
       the mechanical transmission, matching Ctrl_Kinematics_ToMotorUnits(). */
    static const float direction[HOME_AXIS_COUNT] = {1.0f, -1.0f, -1.0f};
    return g_home_pose.joint_deg[axis] + direction[axis] * motor_deg / GEAR_RATIO;
}

bool HomePose_IsJointAngleSafe(HomeAxis_t axis, float joint_deg)
{
    if (axis >= HOME_AXIS_COUNT)
        return false;

    const float tol = ACTUAL_JOINT_LIMIT_TOLERANCE_DEG;
    switch (axis) {
    case HOME_AXIS_ROT:
        return joint_deg >= (ROT_MIN_DEG - tol) && joint_deg <= (ROT_MAX_DEG + tol);
    case HOME_AXIS_LOW:
        return joint_deg >= (LOW_MIN_DEG - tol) && joint_deg <= (LOW_MAX_DEG + tol);
    case HOME_AXIS_HIGH:
        return joint_deg >= (HIGH_MIN_DEG - tol) && joint_deg <= (HIGH_MAX_DEG + tol);
    default:
        return false;
    }
}
