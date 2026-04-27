#ifndef CMD_EXECUTOR_H
#define CMD_EXECUTOR_H

#include "gcode_parser.h"

void Cmd_Executor_Init(float start_x, float start_y, float start_z);
void Cmd_Executor_Run(const GCodeFrame_t *frame);

#endif /* CMD_EXECUTOR_H */
