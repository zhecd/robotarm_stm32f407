#ifndef GCODE_PARSER_H
#define GCODE_PARSER_H

#include <stdint.h>
#include <stdbool.h>

// 定义系统支持的 G 代码和 M 代码枚举
typedef enum {
    GCMD_UNKNOWN = -1,
    GCMD_G0 = 0, // 快速移动/直线移动
    GCMD_G1 = 1, // 直线插补
    GCMD_M3 = 3, // 张开夹爪
    GCMD_M5 = 5  // 闭合夹爪
} GCodeType_t;

// G 代码解析后的数据帧结构体
typedef struct {
    GCodeType_t type; // 指令类型
    float x;          // X轴目标坐标
    float y;          // Y轴目标坐标
    float z;          // Z轴目标坐标
    uint32_t f;       // 运动速度 (Feedrate)
    
    // 标记位：记录指令中是否显式包含了这些轴的信息
    bool has_x;
    bool has_y;
    bool has_z;
    bool has_f;
} GCodeFrame_t;

// API 函数声明：解析传入的一行字符串
bool GCode_ParseLine(const char* line, GCodeFrame_t* out_frame);

#endif /* GCODE_PARSER_H */