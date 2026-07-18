/**
 * @file    kinematics.c
 * @brief   Inverse kinematics implementation for parallel-gripper SCARA arm. / 楠炲疇顢戞径鍦焻 SCARA 閺堢儤顫懛鍌炩偓鍡氱箥閸斻劌顒熺€圭偟骞囬妴? * @ingroup control
 */

#include "algorithm/kinematics.h"
#include "robot_math.h"
#include <stdbool.h>

#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

void Kinematics_Init(void)
{
    /* No pre-computation needed for this structure. / 濮濄倗绮ㄩ弸鍕￥闂団偓妫板嫯顓哥粻妞尖偓?*/
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

ErrorCode_t Kinematics_Solve(float x, float y, float z, RobotAngles_t *angles)
{
    if (!angles) return ERR_NULL_PARAM;
    if (!isfinite(x) || !isfinite(y) || !isfinite(z)) return ERR_OUT_OF_RANGE;

    /* 閳光偓閳光偓 Coordinate transform: desktop 閳?shoulder frame / 閸ф劖鐖ｉ崣妯诲床: 濡楀矂娼?閳?閼测晠鍎撮崸鎰垼缁?閳光偓閳光偓 */
    float zi = z - BASE_HEIGHT;

    /* 閳光偓閳光偓 鑳?: Base rotation / 鎼存洖楠囬弮瀣祮 閳光偓閳光偓 */
    angles->rot = atan2f(x, y);

    /* 閳光偓閳光偓 Compute wrist position from fingertip + tool offset / 娴犲孩瀵氱亸?瀹搞儱鍙块崑蹇曅╃拋锛勭暬閼垫洟鍎存担宥囩枂 閳光偓閳光偓 */
    float r_target = sqrtf(x * x + y * y);
    float r_wrist  = r_target - TOOL_OFFSET_R;
    float z_wrist  = zi - TOOL_OFFSET_Z;
    if (r_wrist < 0.0f) return ERR_OUT_OF_RANGE;

    /* 閳光偓閳光偓 Standard 2-link IK (L1 == L2 = 140 mm, isosceles) / 閺嶅洤鍣禍宀冪箾閺夊棝鈧棜袙 (L1==L2=140mm, 缁涘鍙? 閳光偓閳光偓 */
    float dist_sq = r_wrist * r_wrist + z_wrist * z_wrist;
    float dist    = sqrtf(dist_sq);
    if (dist > (LINK_1_LEN + LINK_2_LEN) ||
        dist < fabsf(LINK_1_LEN - LINK_2_LEN))
        return ERR_OUT_OF_RANGE;

    float val      = dist / (2.0f * LINK_1_LEN);
    float alpha_rad = acosf(CLAMP(val, -1.0f, 1.0f));
    float phi_rad   = atan2f(r_wrist, z_wrist);

    /* 閳光偓閳光偓 鑳?: Shoulder / 閼测晛鍙ч懞?閳光偓閳光偓 */
    angles->low = phi_rad - alpha_rad;

    /* 閳光偓閳光偓 鑳?: Elbow / 閼叉ê鍙ч懞?閳光偓閳光偓 */
    float elbow_r = LINK_1_LEN * sinf(angles->low);
    float elbow_z = LINK_1_LEN * cosf(angles->low);
    float dr = r_wrist - elbow_r;
    float dz = z_wrist - elbow_z;
    angles->high = atan2f(dz, dr);

    /* 閳光偓閳光偓 Rad 閳?Deg / 瀵冨鏉烆剝顫楁惔?閳光偓閳光偓 */
    angles->rot  = RAD_TO_DEG(angles->rot);
    angles->low  = RAD_TO_DEG(angles->low);
    angles->high = RAD_TO_DEG(angles->high);
    if (!AnglesWithinJointLimits(angles)) return ERR_OUT_OF_RANGE;
    return ERR_OK;
}

void Kinematics_ToMotorUnits(const RobotAngles_t *angles, RobotMotorUnits_t *units)
{
    if (!angles || !units) return;

    units->rot_units  =  (int32_t)(angles->rot  * UNITS_PER_DEGREE);
    units->low_units  = -(int32_t)(angles->low  * UNITS_PER_DEGREE);
    units->high_units = -(int32_t)(angles->high * UNITS_PER_DEGREE);
}
