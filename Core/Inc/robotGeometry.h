#ifndef ROBOT_GEOMETRY_H
#define ROBOT_GEOMETRY_H

#include "main.h"
#include <math.h>
#include <stdint.h>

/*
 * Robot geometry configuration
 * Unit: millimeters unless otherwise noted.
 */
#define LINK_1_LEN    140.0f
#define LINK_2_LEN    140.0f
#define TOOL_OFFSET_R 45.0f
#define TOOL_OFFSET_Z -40.0f
#define BASE_HEIGHT   140.0f

#define GEAR_RATIO       4.5f
#define OFFSET_ROT       0.0f
#define OFFSET_LOW       0.0f
#define OFFSET_HIGH      0.0f
#define UNITS_PER_DEGREE (GEAR_RATIO * 10.0f)
#define RAD_TO_DEG(x)    ((x) * 57.2957795f)

typedef struct
{
    float rot;
    float low;
    float high;
} RobotAngles;

typedef struct
{
    int32_t rotUnits;
    int32_t lowUnits;
    int32_t highUnits;
} RobotMotorUnits;

void RobotGeometry_Init(void);
void RobotGeometry_CalculateAngles(float x, float y, float z, RobotAngles *angles);
void RobotGeometry_AnglesToMotorUnits(RobotAngles *angles, RobotMotorUnits *units);

#endif /* ROBOT_GEOMETRY_H */
