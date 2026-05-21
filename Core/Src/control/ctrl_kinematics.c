/**
 * @file    ctrl_kinematics.c
 * @brief   Inverse kinematics implementation for parallel-gripper SCARA arm. / 平行夹爪 SCARA 机械臂逆运动学实现。
 * @ingroup control
 */

#include "control/ctrl_kinematics.h"
#include "common.h"

#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

void Ctrl_Kinematics_Init(void)
{
    /* No pre-computation needed for this structure. / 此结构无需预计算。 */
}

void Ctrl_Kinematics_Solve(float x, float y, float z, RobotAngles_t *angles)
{
    if (!angles) return;

    /* ── Coordinate transform: desktop → shoulder frame / 坐标变换: 桌面 → 肩部坐标系 ── */
    float zi = z - BASE_HEIGHT;

    /* ── θ1: Base rotation / 底座旋转 ── */
    angles->rot = atan2f(x, y);

    /* ── Compute wrist position from fingertip + tool offset / 从指尖+工具偏移计算腕部位置 ── */
    float r_target = sqrtf(x * x + y * y);
    float r_wrist  = r_target - TOOL_OFFSET_R;
    float z_wrist  = zi - TOOL_OFFSET_Z;
    if (r_wrist < 1.0f) r_wrist = 1.0f;

    /* ── Standard 2-link IK (L1 == L2 = 140 mm, isosceles) / 标准二连杆逆解 (L1==L2=140mm, 等腰) ── */
    float dist_sq = r_wrist * r_wrist + z_wrist * z_wrist;
    float dist    = sqrtf(dist_sq);
    if (dist > (LINK_1_LEN + LINK_2_LEN))
        dist = LINK_1_LEN + LINK_2_LEN;

    float val      = dist / (2.0f * LINK_1_LEN);
    float alpha_rad = acosf(CLAMP(val, -1.0f, 1.0f));
    float phi_rad   = atan2f(r_wrist, z_wrist);

    /* ── θ2: Shoulder / 肩关节 ── */
    angles->low = phi_rad - alpha_rad;

    /* ── θ3: Elbow / 肘关节 ── */
    float elbow_r = LINK_1_LEN * sinf(angles->low);
    float elbow_z = LINK_1_LEN * cosf(angles->low);
    float dr = r_wrist - elbow_r;
    float dz = z_wrist - elbow_z;
    angles->high = atan2f(dz, dr);

    /* ── Rad → Deg / 弧度转角度 ── */
    angles->rot  = RAD_TO_DEG(angles->rot);
    angles->low  = RAD_TO_DEG(angles->low);
    angles->high = RAD_TO_DEG(angles->high);
}

void Ctrl_Kinematics_ToMotorUnits(const RobotAngles_t *angles, RobotMotorUnits_t *units)
{
    if (!angles || !units) return;

    units->rot_units  =  (int32_t)(angles->rot  * UNITS_PER_DEGREE);
    units->low_units  = -(int32_t)(angles->low  * UNITS_PER_DEGREE);
    units->high_units = -(int32_t)(angles->high * UNITS_PER_DEGREE);
}
