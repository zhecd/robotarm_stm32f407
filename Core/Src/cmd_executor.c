#include "cmd_executor.h"
#include "motion_planner.h"
#include "bsp_gripper.h" // 引入我们刚刚封装好的夹爪驱动
#include <stdio.h>
#include <math.h>

// 维护机械臂的当前坐标（即上一次运动的终点）
static float current_x = 0.0f;
static float current_y = 0.0f;
static float current_z = 0.0f;

// 默认进给率 (Feedrate)，单位：mm/min (比如 F3000 代表每分钟移动 3000mm)
static float current_feedrate = 3000.0f; 

/**
  * @brief  初始化指令执行器，同步机械臂的初始坐标
  */
void Cmd_Executor_Init(float start_x, float start_y, float start_z) {
    current_x = start_x;
    current_y = start_y;
    current_z = start_z;
}

/**
  * @brief  运行解析好的 G 代码帧
  */
void Cmd_Executor_Run(const GCodeFrame_t* frame) {
    if (frame == NULL) return;

    // ==========================================
    // 1. 处理空间直线运动 (G0 快速移动 / G1 线性插补)
    // ==========================================
    if (frame->type == GCMD_G0 || frame->type == GCMD_G1) {
        
        // 解析新的目标坐标：如果有新坐标就使用新坐标，没有就保持在原地
        float target_x = frame->has_x ? frame->x : current_x;
        float target_y = frame->has_y ? frame->y : current_y;
        float target_z = frame->has_z ? frame->z : current_z;
        
        // 更新运动速度 (F值)，单位 mm/min
        if (frame->has_f && frame->f > 0) {
            current_feedrate = (float)frame->f;
        }

        // 计算起点到终点的三维空间直线距离 (毫米)
        float dx = target_x - current_x;
        float dy = target_y - current_y;
        float dz = target_z - current_z;
        float distance = sqrtf(dx * dx + dy * dy + dz * dz);

        // 根据距离和速度，计算出真正的运动耗时 (毫秒)
        uint32_t duration_ms = 0;
        if (distance > 0.001f) { 
            duration_ms = (uint32_t)((distance * 60000.0f) / current_feedrate);
            if (duration_ms == 0) duration_ms = 1; // 安全限制，防止除零
        }

        // 将计算好的安全坐标和真实耗时，塞入底层轨迹规划器
        Motion_Planner_MoveLine(target_x, target_y, target_z, duration_ms);

        // 更新当前坐标记录，为下一条线段做准备
        current_x = target_x;
        current_y = target_y;
        current_z = target_z;
    }
    // ==========================================
    // 2. 处理外设控制 (M代码 - 舵机夹爪)
    // ==========================================
    else if (frame->type == GCMD_M3) {
        // M3：张开夹爪
        BSP_Gripper_Open(&hgripper);
    }
    else if (frame->type == GCMD_M5) {
        // M5：闭合夹爪
        BSP_Gripper_Close(&hgripper);
    }
}