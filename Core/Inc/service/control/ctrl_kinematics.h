/**
 * @file    ctrl_kinematics.h
 * @brief   Inverse kinematics for a 3-DOF SCARA-like parallel-gripper arm / 3-DOF SCARA 平行夹爪臂逆运动学
 * @ingroup control
 *
 * Link geometry / 连杆几何:
 *   - Link 1 (shoulder / 肩):  140 mm
 *   - Link 2 (elbow / 肘):     140 mm
 *   - Tool offset / 工具偏移:   45 mm horizontal / 水平, -40 mm vertical / 垂直
 *   - Base height / 底座高度:   140 mm (desktop to shoulder / 桌面到肩)
 *
 * G-code coordinates use a desktop frame: Z=0 at table surface, Z+ upward.
 * G-code 坐标使用桌面坐标系: 桌面 Z=0, 上方为 Z+。
 */

#ifndef __CTRL_KINEMATICS_H__
#define __CTRL_KINEMATICS_H__

#include <math.h>
#include <stdint.h>
#include "error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float rot;      /* Base rotation angle (deg) / 底座旋转角度 */
    float low;      /* Shoulder angle (deg) / 肩关节角度 */
    float high;     /* Elbow angle (deg) / 肘关节角度 */
} RobotAngles_t;

typedef struct {
    int32_t rot_units;   /* M1 motor units / M1 电机单位 */
    int32_t low_units;   /* M2 motor units / M2 电机单位 */
    int32_t high_units;  /* M3 motor units / M3 电机单位 */
} RobotMotorUnits_t;

void Ctrl_Kinematics_Init(void);

/** Solve inverse kinematics for a Cartesian target / 逆运动学求解笛卡尔坐标目标 */
/* Returns ERR_OUT_OF_RANGE when the target cannot be reached by the arm. */
ErrorCode_t Ctrl_Kinematics_Solve(float x, float y, float z, RobotAngles_t *angles);

/** Convert joint angles to motor step units / 关节角度转电机步数单位 */
void Ctrl_Kinematics_ToMotorUnits(const RobotAngles_t *angles, RobotMotorUnits_t *units);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_KINEMATICS_H__ */
