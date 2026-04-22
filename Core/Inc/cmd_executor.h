#ifndef CMD_EXECUTOR_H
#define CMD_EXECUTOR_H

#include "gcode_parser.h"

// 传入机械臂的初始默认位置系
void Cmd_Executor_Init(float start_x, float start_y, float start_z);
// 执行解析好的帧
void Cmd_Executor_Run(const GCodeFrame_t* frame);

#endif