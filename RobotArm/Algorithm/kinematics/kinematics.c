/**
 * @file kinematics.c
 * @brief 逆运动学：桌面坐标转换为关节角，再转换为有符号电机微步数。
 * @details 当前逆运动学公式假设 LINK_1_LEN == LINK_2_LEN；更换机构参数需重新核对公式。
 */

#include "algorithm/kinematics.h"
#include "robot_math.h"
#include <stdbool.h>

#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

void Kinematics_Init(void)
{
    /* No pre-computation needed for this structure. */
}

static bool AnglesWithinJointLimits(const RobotAngles_t *angles)
{
#if JOINT_LIMITS_ENABLED
    return angles->rot  >= ROT_MIN_DEG  && angles->rot  <= ROT_MAX_DEG &&
           angles->low  >= LOW_MIN_DEG  && angles->low  <= LOW_MAX_DEG &&
           angles->high >= HIGH_MIN_DEG && angles->high <= HIGH_MAX_DEG;
#else
    (void)angles;
    return true;
#endif
}

/** 输入桌面坐标 x/y/z（mm），输出 rot/low/high（关节 deg）；不可达或越限返回错误。 */
ErrorCode_t Kinematics_Solve(float x, float y, float z, RobotAngles_t *angles)
{
    if (!angles) return ERR_NULL_PARAM;
    if (!isfinite(x) || !isfinite(y) || !isfinite(z)) return ERR_OUT_OF_RANGE;

    /* 坐标变换：桌面高度减去底座高度，得到肩关节参考坐标。 */
    float zi = z - BASE_HEIGHT;

    /* 底座转角：以正 Y 为零方向，atan2 的参数顺序为 (x, y)。 */
    angles->rot = atan2f(x, y);

    /* 根据末端目标和工具偏移，求腕部目标位置。 */
    float r_target = sqrtf(x * x + y * y);
    float r_wrist  = r_target - TOOL_OFFSET_R;
    float z_wrist  = zi - TOOL_OFFSET_Z;
    if (r_wrist < 0.0f) return ERR_OUT_OF_RANGE;

    /* 在等长双连杆假设下求解平面逆运动学。 */
    float dist_sq = r_wrist * r_wrist + z_wrist * z_wrist;
    float dist    = sqrtf(dist_sq);
    if (dist > (LINK_1_LEN + LINK_2_LEN) ||
        dist < fabsf(LINK_1_LEN - LINK_2_LEN))
        return ERR_OUT_OF_RANGE;

    float val      = dist / (2.0f * LINK_1_LEN);
    float alpha_rad = acosf(CLAMP(val, -1.0f, 1.0f));
    float phi_rad   = atan2f(r_wrist, z_wrist);

    /* 大臂关节角。 */
    angles->low = phi_rad - alpha_rad;

    /* 小臂姿态角。 */
    float elbow_r = LINK_1_LEN * sinf(angles->low);
    float elbow_z = LINK_1_LEN * cosf(angles->low);
    float dr = r_wrist - elbow_r;
    float dz = z_wrist - elbow_z;
    angles->high = atan2f(dz, dr);

    /* 将弧度换算为角度（deg）。 */
    angles->rot  = RAD_TO_DEG(angles->rot);
    angles->low  = RAD_TO_DEG(angles->low);
    angles->high = RAD_TO_DEG(angles->high);
    if (!AnglesWithinJointLimits(angles)) return ERR_OUT_OF_RANGE;
    return ERR_OK;
}

/** 关节 deg 转电机微步；M2/M3 取反，不能直接用电机轴角度换算替代。 */
void Kinematics_ToMotorUnits(const RobotAngles_t *angles, RobotMotorUnits_t *units)
{
    if (!angles || !units) return;

    units->rot_units  =  (int32_t)(angles->rot  * UNITS_PER_DEGREE);
    units->low_units  = -(int32_t)(angles->low  * UNITS_PER_DEGREE);
    units->high_units = -(int32_t)(angles->high * UNITS_PER_DEGREE);
}
