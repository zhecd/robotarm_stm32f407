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

bool App_GCodeParser_ParseLine(const char *line, GCodeFrame_t *frame)
{
    if (!line || !frame) return false;

    ResetFrame(frame);
    bool valid = false;
    const char *c = line;

    while (*c) {
        if (isspace((unsigned char)*c)) { c++; continue; }

        char letter = (char)toupper((unsigned char)*c++);
        char *end = NULL;

        switch (letter) {
        case 'G': {
            long g = strtol(c, &end, 10);
            if (end == c) return false;
            if (g == 0)      { frame->type = GCMD_G0; valid = true; }
            else if (g == 1) { frame->type = GCMD_G1; valid = true; }
            else return false;
            c = end;
            continue;
        }
        case 'M': {
            long m = strtol(c, &end, 10);
            if (end == c) return false;
            if (m == 3)      { frame->type = GCMD_M3; valid = true; }
            else if (m == 5) { frame->type = GCMD_M5; valid = true; }
            else if (m == 999) { frame->type = GCMD_M999; valid = true; }
            else return false;
            c = end;
            continue;
        }
        case 'X':
            frame->x = strtof(c, &end); if (end == c) return false;
            frame->has_x = true; c = end; continue;
        case 'Y':
            frame->y = strtof(c, &end); if (end == c) return false;
            frame->has_y = true; c = end; continue;
        case 'Z':
            frame->z = strtof(c, &end); if (end == c) return false;
            frame->has_z = true; c = end; continue;
        case 'F': {
            unsigned long feed = strtoul(c, &end, 10);
            if (end == c || feed == 0U) return false;
            frame->f = (uint32_t)feed; frame->has_f = true; c = end; continue;
        }
        default: return false;
        }
    }

    if ((frame->type == GCMD_M3 || frame->type == GCMD_M5) &&
        (frame->has_x || frame->has_y || frame->has_z || frame->has_f))
        return false;
    return valid;
}
