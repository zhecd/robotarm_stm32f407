/**
 * @file    app_gcode_parser.h
 * @brief   Lightweight G-code line parser (G0/G1/M3/M5) / 轻量 G-code 行解析器 (G0/G1/M3/M5)
 * @ingroup app
 *
 * Parses a single line of G-code into a GCodeFrame_t.
 * Supported commands / 支持命令: G0, G1 (linear moves / 直线运动), M3 (gripper open / 夹爪开), M5 (gripper close / 夹爪合)
 * Parameters / 参数: X, Y, Z, F (feedrate / 进给速度 mm/min)
 */

#ifndef __APP_GCODE_PARSER_H__
#define __APP_GCODE_PARSER_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GCMD_UNKNOWN = -1,      /* Unknown command / 未知指令 */
    GCMD_G0 = 0,            /* Rapid linear move / 快速直线移动 */
    GCMD_G1 = 1,            /* Linear move with feedrate / 带进给速度直线移动 */
    GCMD_M999,              /* Fault recovery: re-home and recalibrate. */
    GCMD_M3 = 3,            /* Gripper open / 夹爪张开 */
    GCMD_M5 = 5             /* Gripper close / 夹爪闭合 */
} GCodeType_t;

typedef struct {
    GCodeType_t type;       /* Command type / 指令类型 */
    float       x, y, z;    /* Target coordinates / 目标坐标 */
    uint32_t    f;          /* Feedrate (mm/min) / 进给速度 */
    bool        has_x, has_y, has_z, has_f;  /* Parameter present flags / 参数存在标志 */
} GCodeFrame_t;

/** Parse one line of G-code / 解析一行 G-code */
bool App_GCodeParser_ParseLine(const char *line, GCodeFrame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __APP_GCODE_PARSER_H__ */
