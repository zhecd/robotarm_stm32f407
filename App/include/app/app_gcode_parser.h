/**
 * @file    app_gcode_parser.h
 * @brief   Lightweight G-code line parser (G0/G1/M3/M5) / 鏉炲鍣?G-code 鐞涘矁袙閺嬫劕娅?(G0/G1/M3/M5)
 * @ingroup app
 *
 * Parses a single line of G-code into a GCodeFrame_t.
 * Supported commands / 閺€顖涘瘮閸涙垝鎶? G0, G1 (linear moves / 閻╁鍤庢潻鎰З), M3 (gripper open / 婢跺湱鍩呭鈧?, M5 (gripper close / 婢跺湱鍩呴崥?
 * Parameters / 閸欏倹鏆? X, Y, Z, F (feedrate / 鏉╂稓绮伴柅鐔峰 mm/min)
 */

#ifndef __APP_GCODE_PARSER_H__
#define __APP_GCODE_PARSER_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GCMD_UNKNOWN = -1,      /* Unknown command / 閺堫亞鐓￠幐鍥︽姢 */
    GCMD_G0 = 0,            /* Rapid linear move / 韫囶偊鈧喓娲跨痪璺ㄐ╅崝?*/
    GCMD_G1 = 1,            /* Linear move with feedrate / 鐢箒绻樼紒娆撯偓鐔峰閻╁鍤庣粔璇插З */
    GCMD_M3 = 3,            /* Gripper open / 婢跺湱鍩呭鐘茬磻 */
    GCMD_M5 = 5,            /* Gripper close / 婢跺湱鍩呴梻顓炴値 */
    GCMD_M114 = 114,        /* Planned position and encoder status. */
    GCMD_M119 = 119,        /* Limit switch state. */
    GCMD_M400 = 400,        /* Wait until motion queue is empty. */
    GCMD_M999 = 999         /* Fault recovery: re-home and recalibrate. */
} GCodeType_t;

typedef struct {
    GCodeType_t type;       /* Command type / 閹稿洣鎶ょ猾璇茬€?*/
    float       x, y, z;    /* Target coordinates / 閻╊喗鐖ｉ崸鎰垼 */
    uint32_t    f;          /* Feedrate (mm/min) / 鏉╂稓绮伴柅鐔峰 */
    bool        has_x, has_y, has_z, has_f;  /* Parameter present flags / 閸欏倹鏆熺€涙ê婀弽鍥х箶 */
} GCodeFrame_t;

/** Parse one line of G-code / 鐟欙絾鐎芥稉鈧悰?G-code */
bool App_GCodeParser_ParseLine(const char *line, GCodeFrame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __APP_GCODE_PARSER_H__ */
