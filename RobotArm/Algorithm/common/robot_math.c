/**
 * @file robot_math.c
 * @brief 公共数学工具：角度换算和三轴绝对步数比较。
 * @details StepsToDeg/DegToSteps 使用电机轴角度；关节角换微步应使用 UNITS_PER_DEGREE。
 */

#include "robot_math.h"
#include <stdlib.h>

uint32_t RobotMath_MaxAbs3(int32_t a, int32_t b, int32_t c)
{
    /* 先扩展到 64 位再取绝对值，INT32_MIN 的正值无法用 int32_t 表示。 */
    uint32_t max_val = (uint32_t)llabs((long long)a);
    uint32_t abs_b   = (uint32_t)llabs((long long)b);
    uint32_t abs_c   = (uint32_t)llabs((long long)c);
    if (abs_b > max_val) max_val = abs_b;
    if (abs_c > max_val) max_val = abs_c;
    return max_val;
}
