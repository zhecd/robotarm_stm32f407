#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "app/app_gcode_parser.h"
#include "algorithm/kinematics.h"
#include "error_code.h"
#include "robot_config.h"
#include "robot_math.h"

static void TestGCodeParser(void)
{
    GCodeFrame_t frame;

    assert(App_GCodeParser_ParseLine("G1 X100 Y180 Z160 F2000", &frame));
    assert(frame.type == GCMD_G1);
    assert(frame.has_x && frame.has_y && frame.has_z && frame.has_f);
    assert(fabsf(frame.x - 100.0f) < 0.001f);
    assert(frame.f == 2000U);

    assert(App_GCodeParser_ParseLine("m3", &frame));
    assert(frame.type == GCMD_M3);
    assert(App_GCodeParser_ParseLine("M400", &frame));
    assert(frame.type == GCMD_M400);

    assert(!App_GCodeParser_ParseLine("G1 X10 X20", &frame));
    assert(!App_GCodeParser_ParseLine("M5 X1", &frame));
    assert(!App_GCodeParser_ParseLine("G1", &frame));
}

static void TestKinematics(void)
{
    RobotAngles_t angles;
    RobotMotorUnits_t units;

    assert(Kinematics_Solve(HOMEPOSE_X_MM, HOMEPOSE_Y_MM, HOMEPOSE_Z_MM, &angles) == ERR_OK);
    assert(fabsf(angles.rot - HOMEPOSE_ROT_DEG) < 0.01f);
    assert(fabsf(angles.low - HOMEPOSE_LOW_DEG) < 0.01f);
    assert(fabsf(angles.high - HOMEPOSE_HIGH_DEG) < 0.01f);

    assert(Kinematics_Solve(0.0f, 185.0f, 1000.0f, &angles) == ERR_OUT_OF_RANGE);
    assert(Kinematics_Solve(NAN, 185.0f, 240.0f, &angles) == ERR_OUT_OF_RANGE);

    angles.rot = 10.0f;
    angles.low = -5.0f;
    angles.high = 20.0f;
    Kinematics_ToMotorUnits(&angles, &units);
    assert(units.rot_units == (int32_t)(10.0f * UNITS_PER_DEGREE));
    assert(units.low_units == -(int32_t)(-5.0f * UNITS_PER_DEGREE));
    assert(units.high_units == -(int32_t)(20.0f * UNITS_PER_DEGREE));
}

static void TestClosedLoopUnitConvention(void)
{
    /* Kinematics emits motor steps.  A one-degree joint move corresponds to
       GEAR_RATIO motor-shaft degrees and must be directly comparable with
       the AS5600 motor-shaft reading. */
    int32_t motor_steps = (int32_t)UNITS_PER_DEGREE;
    assert(fabsf(StepsToDeg(motor_steps) - GEAR_RATIO) < 0.001f);
}

int main(void)
{
    TestGCodeParser();
    TestKinematics();
    TestClosedLoopUnitConvention();
    puts("robotarm_logic_tests: passed");
    return 0;
}
