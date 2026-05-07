#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include <math.h>

/* ── 步进电机参数 (16细分, 步距角1.8°) ── */
#define GEAR_RATIO          1.0f
#define STEPS_PER_REV       3200.0f          /* 200步 × 16细分 */
#define STEPS_PER_DEGREE    (STEPS_PER_REV / 360.0f)
#define DEGREES_PER_STEP    (360.0f / STEPS_PER_REV)

/* ── 角度工具 ── */

/* 归一化角度到 (-180, 180] */
static inline float AngleWrap180(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

/* ── 步数/角度转换 ── */

static inline float StepsToDeg(int32_t steps)
{
    return (float)steps * DEGREES_PER_STEP;
}

static inline int32_t DegToSteps(float deg)
{
    return (int32_t)roundf(deg * STEPS_PER_DEGREE);
}

/* ── 运动帧工具 (前向声明 motor_core.h 的 MotionFrame_t) ── */
struct MotionFrame_t;  /* 不完整类型, 避免循环依赖 */

uint32_t Common_MaxAbs3(int32_t a, int32_t b, int32_t c);

#endif
