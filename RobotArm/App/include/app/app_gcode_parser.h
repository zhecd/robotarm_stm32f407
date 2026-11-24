/**
 * @file app_gcode_parser.h
 * @brief G-code 语法解析：只生成命令结构，不调用设备或执行运动。
 */

#ifndef __APP_GCODE_PARSER_H__
#define __APP_GCODE_PARSER_H__

#include <stdbool.h>
#include <stdint.h>
#include "command_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Parse one line of G-code */
bool App_GCodeParser_ParseLine(const char *line, GCodeFrame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __APP_GCODE_PARSER_H__ */
