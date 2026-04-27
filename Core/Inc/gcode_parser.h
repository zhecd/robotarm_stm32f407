#ifndef GCODE_PARSER_H
#define GCODE_PARSER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    GCMD_UNKNOWN = -1,
    GCMD_G0 = 0,
    GCMD_G1 = 1,
    GCMD_M3 = 3,
    GCMD_M5 = 5
} GCodeType_t;

typedef struct
{
    GCodeType_t type;
    float x;
    float y;
    float z;
    uint32_t f;
    bool has_x;
    bool has_y;
    bool has_z;
    bool has_f;
} GCodeFrame_t;

bool GCode_ParseLine(const char *line, GCodeFrame_t *out_frame);

#endif /* GCODE_PARSER_H */
