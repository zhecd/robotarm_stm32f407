#include "gcode_parser.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/**
  * @brief  解析一行 G-Code 或 M-Code 字符串
  * @param  line: 接收到的串口字符串 (例如 "G1 X100 Y50 F3000" 或 "M3")
  * @param  out_frame: 解析后输出的结构体
  * @retval true: 解析成功包含有效指令 / false: 解析失败或空指令
  */
bool GCode_ParseLine(const char* line, GCodeFrame_t* out_frame) {
    if (line == NULL || out_frame == NULL) {
        return false;
    }

    // 1. 初始化清空结构体，防止上一次的脏数据残留
    out_frame->type = GCMD_UNKNOWN;
    out_frame->x = 0.0f; out_frame->has_x = false;
    out_frame->y = 0.0f; out_frame->has_y = false;
    out_frame->z = 0.0f; out_frame->has_z = false;
    out_frame->f = 0;    out_frame->has_f = false;

    bool is_valid_cmd = false;
    const char* ptr = line;

    // 2. 遍历解析字符串
    while (*ptr != '\0') {
        // 跳过空格和控制台换行符
        if (isspace((unsigned char)*ptr)) {
            ptr++;
            continue;
        }

        // 提取当前字母，统一转为大写 (这样发 g1 或者 G1 都能识别)
        char letter = toupper((unsigned char)*ptr);
        ptr++; // 指针后移，指向后面的数字部分

        // 3. 根据首字母匹配对应的值
        if (letter == 'G') {
            int g_code = atoi(ptr);
            if (g_code == 0) { out_frame->type = GCMD_G0; is_valid_cmd = true; }
            else if (g_code == 1) { out_frame->type = GCMD_G1; is_valid_cmd = true; }
        } 
        else if (letter == 'M') {
            int m_code = atoi(ptr);
            // ★ 这里就是新增的 M3 和 M5 解析逻辑 ★
            if (m_code == 3) { out_frame->type = GCMD_M3; is_valid_cmd = true; }
            else if (m_code == 5) { out_frame->type = GCMD_M5; is_valid_cmd = true; }
        } 
        else if (letter == 'X') {
            out_frame->x = atof(ptr); // atof 支持解析带小数点的浮点数
            out_frame->has_x = true;
        } 
        else if (letter == 'Y') {
            out_frame->y = atof(ptr);
            out_frame->has_y = true;
        } 
        else if (letter == 'Z') {
            out_frame->z = atof(ptr);
            out_frame->has_z = true;
        } 
        else if (letter == 'F') {
            out_frame->f = (uint32_t)atoi(ptr);
            out_frame->has_f = true;
        }

        // 4. 跳过当前的数字部分，寻找下一个指令字母 (支持数字、小数点、正负号)
        while (*ptr != '\0' && (isdigit((unsigned char)*ptr) || *ptr == '.' || *ptr == '-' || *ptr == '+')) {
            ptr++;
        }
    }

    return is_valid_cmd;
}