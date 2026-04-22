#include "cmd_executor.h"
#include "motion_planner.h"
#include <stdio.h>

// 维护机械臂的当前目标坐标
static float current_target_x = 0.0f;
static float current_target_y = 0.0f;
static float current_target_z = 0.0f;
static uint32_t default_duration = 2000; // 默认 2000ms

void Cmd_Executor_Init(float start_x, float start_y, float start_z) {
    current_target_x = start_x;
    current_target_y = start_y;
    current_target_z = start_z;
}

void Cmd_Executor_Run(const GCodeFrame_t* frame) {
    if (frame == NULL) return;

    if (frame->type == GCMD_G0 || frame->type == GCMD_G1) {
        
        // 增量更新坐标：如果有新坐标就覆盖，没有就保持原来的目标
        if (frame->has_x) current_target_x = frame->x;
        if (frame->has_y) current_target_y = frame->y;
        if (frame->has_z) current_target_z = frame->z;
        if (frame->has_f) default_duration = frame->f;

        // 安全检查与调用你的底层规划器
        // Motion_Planner_MoveLine 内部有阻塞队列判断，非常安全
        Motion_Planner_MoveLine(current_target_x, 
                                current_target_y, 
                                current_target_z, 
                                default_duration);
    }
    // else if (frame->type == 某些 M 指令) { 处理夹爪等 }
}