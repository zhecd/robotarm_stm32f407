/**
 * @file    app_gcode_parser.c
 * @brief   G-code line parser implementation. / G-code 行解析器实现。
 * @ingroup app
 */

#include "app/app_gcode_parser.h"
#include <ctype.h>
#include <math.h>
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
    const char *c = line;

    while (isspace((unsigned char)*c)) c++;
    char command_letter = (char)toupper((unsigned char)*c++);
    char *end = NULL;
    long command = 0;

    /* Exactly one leading G/M command is accepted.  Parameters can only
       follow it; a second G/M is rejected rather than silently overriding. */
    if (command_letter == 'G') {
        command = strtol(c, &end, 10);
        if (end == c) return false;
        if (command == 0) frame->type = GCMD_G0;
        else if (command == 1) frame->type = GCMD_G1;
        else return false;
    } else if (command_letter == 'M') {
        command = strtol(c, &end, 10);
        if (end == c) return false;
        if (command == 3) frame->type = GCMD_M3;
        else if (command == 5) frame->type = GCMD_M5;
        else if (command == 999) frame->type = GCMD_M999;
        else return false;
    } else {
        return false;
    }
    c = end;

    while (*c) {
        if (isspace((unsigned char)*c)) { c++; continue; }
        if (command_letter != 'G') return false; /* M3/M5/M999 take no args. */

        char letter = (char)toupper((unsigned char)*c++);
        switch (letter) {
        case 'X':
            if (frame->has_x) return false;
            frame->x = strtof(c, &end);
            if (end == c || !isfinite(frame->x)) return false;
            frame->has_x = true;
            break;
        case 'Y':
            if (frame->has_y) return false;
            frame->y = strtof(c, &end);
            if (end == c || !isfinite(frame->y)) return false;
            frame->has_y = true;
            break;
        case 'Z':
            if (frame->has_z) return false;
            frame->z = strtof(c, &end);
            if (end == c || !isfinite(frame->z)) return false;
            frame->has_z = true;
            break;
        case 'F': {
            if (frame->has_f) return false;
            unsigned long feed = strtoul(c, &end, 10);
            if (end == c || feed == 0U || feed > UINT32_MAX) return false;
            frame->f = (uint32_t)feed;
            frame->has_f = true;
            break;
        }
        default:
            return false;
        }
        c = end;
    }

    return frame->has_x || frame->has_y || frame->has_z || frame->has_f ||
           command_letter == 'M';
}
