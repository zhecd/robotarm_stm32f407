#ifndef GCODE_PARSER_H
#define GCODE_PARSER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    GCMD_UNKNOWN = -1,
    GCMD_G0 = 0, // 快速移动/直线移动
    GCMD_G1 = 1, // 直线插补
    // 后续可以自行添加 M 指令（如 M3 夹爪闭合，M4 夹爪张开）
} GCodeType_t;

typedef struct {
    GCodeType_t type;
    float x;
    float y;
    float z;
    uint32_t f; // 在你的系统里，F 可以复用代表 Duration (ms)
    
    // 标记位：记录指令中是否包含了这些轴的信息
    bool has_x;
    bool has_y;
    bool has_z;
    bool has_f;
} GCodeFrame_t;

// 解析传入的字符串
bool GCode_ParseLine(const char* line, GCodeFrame_t* out_frame);

#endif