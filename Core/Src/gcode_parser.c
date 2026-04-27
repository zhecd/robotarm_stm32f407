#include "gcode_parser.h"
#include <ctype.h>
#include <stdlib.h>

static void GCode_ResetFrame(GCodeFrame_t *frame)
{
    frame->type = GCMD_UNKNOWN;
    frame->x = 0.0f;
    frame->y = 0.0f;
    frame->z = 0.0f;
    frame->f = 0U;
    frame->has_x = false;
    frame->has_y = false;
    frame->has_z = false;
    frame->has_f = false;
}

static void GCode_SkipNumericToken(const char **cursor)
{
    while ((**cursor != '\0') &&
           (isdigit((unsigned char)**cursor) || (**cursor == '.') || (**cursor == '-') || (**cursor == '+')))
    {
        (*cursor)++;
    }
}

bool GCode_ParseLine(const char *line, GCodeFrame_t *out_frame)
{
    if ((line == NULL) || (out_frame == NULL)) {
        return false;
    }

    GCode_ResetFrame(out_frame);

    bool is_valid_cmd = false;
    const char *cursor = line;

    while (*cursor != '\0') {
        if (isspace((unsigned char)*cursor)) {
            cursor++;
            continue;
        }

        char letter = (char)toupper((unsigned char)*cursor);
        cursor++;

        switch (letter) {
        case 'G':
        {
            int g_code = atoi(cursor);
            if (g_code == 0) {
                out_frame->type = GCMD_G0;
                is_valid_cmd = true;
            } else if (g_code == 1) {
                out_frame->type = GCMD_G1;
                is_valid_cmd = true;
            }
            break;
        }

        case 'M':
        {
            int m_code = atoi(cursor);
            if (m_code == 3) {
                out_frame->type = GCMD_M3;
                is_valid_cmd = true;
            } else if (m_code == 5) {
                out_frame->type = GCMD_M5;
                is_valid_cmd = true;
            }
            break;
        }

        case 'X':
            out_frame->x = strtof(cursor, NULL);
            out_frame->has_x = true;
            break;

        case 'Y':
            out_frame->y = strtof(cursor, NULL);
            out_frame->has_y = true;
            break;

        case 'Z':
            out_frame->z = strtof(cursor, NULL);
            out_frame->has_z = true;
            break;

        case 'F':
            out_frame->f = (uint32_t)strtoul(cursor, NULL, 10);
            out_frame->has_f = true;
            break;

        default:
            break;
        }

        GCode_SkipNumericToken(&cursor);
    }

    return is_valid_cmd;
}
