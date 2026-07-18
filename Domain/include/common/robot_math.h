/**
 * @file    common.h
 * @brief   Shared math utilities, angle helpers, and generic macros / 鍏变韩鏁板宸ュ叿銆佽搴﹁緟鍔╁嚱鏁颁笌閫氱敤�?
 * @ingroup common
 *
 * Motor/kinematic configuration lives in robot_config.h / 鐢垫満涓庤繍鍔ㄥ閰嶇疆瑙?robot_config.h
 * Error codes live in error_code.h / 閿欒鐮佸畾涔夎�?error_code.h
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include <math.h>
#include "robot_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 鈹€鈹€ Generic utility macros / 閫氱敤宸ュ叿�?鈹€鈹€ */

#ifndef CLAMP
#define CLAMP(x, lo, hi)  (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))  /* Clamp value to range / 闄愬埗鍊煎埌鑼冨�?*/
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG(r)     ((r) * 57.29577951308232f)   /* Radians to degrees / 寮у害杞�?*/
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD(d)     ((d) * 0.017453292519943295f) /* Degrees to radians / 搴﹁浆寮у�?*/
#endif

/* 鈹€鈹€ Angle utilities / 瑙掑害宸ュ叿 鈹€鈹€ */

/** Normalize an angle to (-180, 180] degrees / 褰掍竴鍖栬搴﹀�?(-180, 180] �?*/
static inline float AngleWrap180(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

/* 鈹€鈹€ Step / degree conversions / 姝ユ�?瑙掑害杞崲 鈹€鈹€ */

/** Convert motor-step count to joint degrees / 鐢垫満姝ユ暟杞叧鑺傝�?*/
static inline float StepsToDeg(int32_t steps)
{
    return (float)steps * DEGREES_PER_STEP;
}

/** Convert joint degrees to motor-step count / 鍏宠妭瑙掑害杞數鏈烘�?*/
static inline int32_t DegToSteps(float deg)
{
    return (int32_t)roundf(deg * STEPS_PER_DEGREE);
}

/* 鈹€鈹€ Math helpers / 鏁板杈呭姪 鈹€鈹€ */

/** Return the maximum absolute value among three int32_t values / 杩斿洖涓変釜鍊肩殑鏈€澶х粷瀵瑰�?*/
uint32_t RobotMath_MaxAbs3(int32_t a, int32_t b, int32_t c);

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_H__ */
