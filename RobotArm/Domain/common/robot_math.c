/**
 * @file    common.c
 * @brief   Shared utility implementations / 閸忓彉闊╁銉ュ徔鐎圭偟骞?
 * @ingroup common
 */

#include "robot_math.h"
#include <stdlib.h>

uint32_t RobotMath_MaxAbs3(int32_t a, int32_t b, int32_t c)
{
    uint32_t max_val = (uint32_t)labs(a);
    uint32_t abs_b   = (uint32_t)labs(b);
    uint32_t abs_c   = (uint32_t)labs(c);
    if (abs_b > max_val) max_val = abs_b;
    if (abs_c > max_val) max_val = abs_c;
    return max_val;
}
