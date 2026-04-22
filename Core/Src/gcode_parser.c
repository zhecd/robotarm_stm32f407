#include "gcode_parser.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

bool GCode_ParseLine(const char* line, GCodeFrame_t* out_frame) {
    if (line == NULL || out_frame == NULL || strlen(line) == 0) return false;

    // 清空结构体默认状态
    out_frame->type = GCMD_UNKNOWN;
    out_frame->has_x = false;
    out_frame->has_y = false;
    out_frame->has_z = false;
    out_frame->has_f = false;

    char* ptr = (char*)line;
    while (*ptr != '\0') {
        if (*ptr == ' ' || *ptr == '\t') {
            ptr++;
            continue;
        }

        char cmd_letter = toupper((unsigned char)*ptr);
        ptr++; // 移动到数字部分
        
        switch (cmd_letter) {
            case 'G':
                out_frame->type = (GCodeType_t)strtol(ptr, &ptr, 10);
                break;
            case 'X':
                out_frame->x = strtof(ptr, &ptr);
                out_frame->has_x = true;
                break;
            case 'Y':
                out_frame->y = strtof(ptr, &ptr);
                out_frame->has_y = true;
                break;
            case 'Z':
                out_frame->z = strtof(ptr, &ptr);
                out_frame->has_z = true;
                break;
            case 'F':
                out_frame->f = strtol(ptr, &ptr, 10);
                out_frame->has_f = true;
                break;
            default:
                // 跳过不认识的参数（或者直接找到下一个空格）
                while(*ptr != ' ' && *ptr != '\0') ptr++;
                break;
        }
    }

    return (out_frame->type != GCMD_UNKNOWN);
}