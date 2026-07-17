/**
 * @file    common.h
 * @brief   Shared math utilities, angle helpers, and generic macros / 共享数学工具、角度辅助函数与通用宏
 * @ingroup common
 *
 * Motor/kinematic configuration lives in robot_config.h / 电机与运动学配置见 robot_config.h
 * Error codes live in error_code.h / 错误码定义见 error_code.h
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include <math.h>
#include "robot_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Generic utility macros / 通用工具宏 ── */

#ifndef CLAMP
#define CLAMP(x, lo, hi)  (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))  /* Clamp value to range / 限制值到范围 */
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG(r)     ((r) * 57.29577951308232f)   /* Radians to degrees / 弧度转度 */
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD(d)     ((d) * 0.017453292519943295f) /* Degrees to radians / 度转弧度 */
#endif

/* ── Angle utilities / 角度工具 ── */

/** Normalize an angle to (-180, 180] degrees / 归一化角度到 (-180, 180] 度 */
static inline float AngleWrap180(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

/* ── Step / degree conversions / 步数/角度转换 ── */

/** Convert motor-step count to joint degrees / 电机步数转关节角度 */
static inline float StepsToDeg(int32_t steps)
{
    return (float)steps * DEGREES_PER_STEP;
}

/** Convert joint degrees to motor-step count / 关节角度转电机步数 */
static inline int32_t DegToSteps(float deg)
{
    return (int32_t)roundf(deg * STEPS_PER_DEGREE);
}

/* ── Math helpers / 数学辅助 ── */

/** Return the maximum absolute value among three int32_t values / 返回三个值的最大绝对值 */
uint32_t Common_MaxAbs3(int32_t a, int32_t b, int32_t c);

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_H__ */
