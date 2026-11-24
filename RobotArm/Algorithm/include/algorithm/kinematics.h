/**
 * @file kinematics.h
 * @brief 逆运动学：桌面坐标转换为关节角，再转换为有符号电机微步数。
 * @details 当前逆运动学公式假设 LINK_1_LEN == LINK_2_LEN；更换机构参数需重新核对公式。
 */

#ifndef ROBOTARM_KINEMATICS_H
#define ROBOTARM_KINEMATICS_H

#include <math.h>
#include <stdint.h>
#include "error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float rot;      /* 底座转角：以正 Y 为零方向，atan2 的参数顺序为 (x, y)。 */
    float low;      /* 大臂关节角。 */
    float high;     /* 小臂姿态角。 */
} RobotAngles_t;

typedef struct {
    int32_t rot_units;   /* M1 motor units */
    int32_t low_units;   /* M2 motor units */
    int32_t high_units;  /* M3 motor units */
} RobotMotorUnits_t;

void Kinematics_Init(void);

/* Solve inverse kinematics for a Cartesian target */
/* Returns ERR_OUT_OF_RANGE when the target cannot be reached by the arm. */
/** 输入桌面坐标 x/y/z（mm），输出 rot/low/high（关节 deg）；不可达或越限返回错误。 */
ErrorCode_t Kinematics_Solve(float x, float y, float z, RobotAngles_t *angles);

/* Convert joint angles to motor step units */
/** 关节 deg 转电机微步；M2/M3 取反，不能直接用电机轴角度换算替代。 */
void Kinematics_ToMotorUnits(const RobotAngles_t *angles, RobotMotorUnits_t *units);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTARM_KINEMATICS_H */
