/**
 * @file    ctrl_kinematics.h
 * @brief   Inverse kinematics for a 3-DOF SCARA-like parallel-gripper arm / 3-DOF SCARA 骞宠澶圭埅鑷傞€嗚繍鍔ㄥ
 * @ingroup control
 *
 * Link geometry / 杩炴潌鍑犱綍:
 *   - Link 1 (shoulder / �?:  140 mm
 *   - Link 2 (elbow / �?:     140 mm
 *   - Tool offset / 宸ュ叿鍋忕�?   45 mm horizontal / 姘村�? -40 mm vertical / 鍨傜�?
 *   - Base height / 搴曞骇楂樺害:   140 mm (desktop to shoulder / 妗岄潰鍒拌偐)
 *
 * G-code coordinates use a desktop frame: Z=0 at table surface, Z+ upward.
 * G-code 鍧愭爣浣跨敤妗岄潰鍧愭爣�? 妗岄�?Z=0, 涓婃柟涓?Z+�?
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
    float rot;      /* Base rotation angle (deg) / 搴曞骇鏃嬭浆瑙掑�?*/
    float low;      /* Shoulder angle (deg) / 鑲╁叧鑺傝�?*/
    float high;     /* Elbow angle (deg) / 鑲樺叧鑺傝�?*/
} RobotAngles_t;

typedef struct {
    int32_t rot_units;   /* M1 motor units / M1 鐢垫満鍗曚�?*/
    int32_t low_units;   /* M2 motor units / M2 鐢垫満鍗曚�?*/
    int32_t high_units;  /* M3 motor units / M3 鐢垫満鍗曚�?*/
} RobotMotorUnits_t;

void Ctrl_Kinematics_Init(void);

/** Solve inverse kinematics for a Cartesian target / 閫嗚繍鍔ㄥ姹傝В绗涘崱灏斿潗鏍囩洰鏍?*/
/* Returns ERR_OUT_OF_RANGE when the target cannot be reached by the arm. */
ErrorCode_t Ctrl_Kinematics_Solve(float x, float y, float z, RobotAngles_t *angles);

/** Convert joint angles to motor step units / 鍏宠妭瑙掑害杞數鏈烘鏁板崟浣?*/
void Ctrl_Kinematics_ToMotorUnits(const RobotAngles_t *angles, RobotMotorUnits_t *units);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_KINEMATICS_H__ */
