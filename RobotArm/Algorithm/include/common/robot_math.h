/**
 * @file robot_math.h
 * @brief 公共数学工具：角度换算和三轴绝对步数比较。
 * @details StepsToDeg/DegToSteps 使用电机轴角度；关节角换微步应使用 UNITS_PER_DEGREE。
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include <math.h>
#include "robot_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 通用数值工具宏。 */

#ifndef CLAMP
#define CLAMP(x, lo, hi)  (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))  /* Clamp value to range */
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG(r)     ((r) * 57.29577951308232f)   /* 将弧度换算为角度（deg）。 */
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD(d)     ((d) * 0.017453292519943295f) /* 将角度（deg）换算为弧度（rad）。 */
#endif

/* 角度工具。 */

/* Normalize an angle to (-180, 180] degrees */
/** 将角度映射到 (-180, 180]；输入应为有限值。 */
static inline float AngleWrap180(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

/* 电机轴角度与微步换算。 */

/* 电机微步数转换为电机轴角度，deg。 */
/** 电机微步转电机轴 deg；不含减速比，也不是关节角。 */
static inline float StepsToDeg(int32_t steps)
{
    return (float)steps * DEGREES_PER_STEP;
}

/* 电机轴角度转换为微步数，不包含减速比。 */
static inline int32_t DegToSteps(float deg)
{
    return (int32_t)roundf(deg * STEPS_PER_DEGREE);
}

/* 数学辅助接口。 */

/* Return the maximum absolute value among three int32_t values */
uint32_t RobotMath_MaxAbs3(int32_t a, int32_t b, int32_t c);

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_H__ */
