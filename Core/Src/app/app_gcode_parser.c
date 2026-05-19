/**
 * @file    app_gcode_parser.c
 * @brief   G-code line parser implementation. / G-code 行解析器实现。
 * @ingroup app
 */

#include "app/app_gcode_parser.h"
#include <ctype.h>
#include <stdlib.h>

static void ResetFrame(GCodeFrame_t *f)
{
    f->type  = GCMD_UNKNOWN;
    f->x = 0.0f; f->y = 0.0f; f->z = 0.0f; f->f = 0U;
    f->has_x = false; f->has_y = false; f->has_z = false; f->has_f = false;
}

static void SkipNumeric(const char **c)
{
    while (**c && (isdigit((unsigned char)**c) || **c == '.' || **c == '-' || **c == '+'))
        (*c)++;
}

bool App_GCodeParser_ParseLine(const char *line, GCodeFrame_t *frame)
{
    if (!line || !frame) return false;

    ResetFrame(frame);
    bool valid = false;
    const char *c = line;

    while (*c) {
        if (isspace((unsigned char)*c)) { c++; continue; }

        char letter = (char)toupper((unsigned char)*c);
        c++;

        switch (letter) {
        case 'G': {
            int g = atoi(c);
            if (g == 0)      { frame->type = GCMD_G0; valid = true; }
            else if (g == 1) { frame->type = GCMD_G1; valid = true; }
            break;
        }
        case 'M': {
            int m = atoi(c);
            if (m == 3)      { frame->type = GCMD_M3; valid = true; }
            else if (m == 5) { frame->type = GCMD_M5; valid = true; }
            break;
        }
        case 'X': frame->x = strtof(c, NULL); frame->has_x = true; break;
        case 'Y': frame->y = strtof(c, NULL); frame->has_y = true; break;
        case 'Z': frame->z = strtof(c, NULL); frame->has_z = true; break;
        case 'F': frame->f = (uint32_t)strtoul(c, NULL, 10); frame->has_f = true; break;
        default: break;
        }
        SkipNumeric(&c);
    }

    return valid;
}
