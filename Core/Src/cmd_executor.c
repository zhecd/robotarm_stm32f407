#include "cmd_executor.h"
#include "motion_planner.h"
#include <stdio.h>
#include <math.h>  // 引入数学库，用于计算 sqrtf (开平方)

// 维护机械臂的当前坐标（即上一次运动的终点）
static float current_x = 0.0f;
static float current_y = 0.0f;
static float current_z = 0.0f;

// 默认进给率 (Feedrate)，单位：mm/min (比如 F3000 代表每分钟移动 3000mm)
static float current_feedrate = 3000.0f; 

void Cmd_Executor_Init(float start_x, float start_y, float start_z) {
    current_x = start_x;
    current_y = start_y;
    current_z = start_z;
}

void Cmd_Executor_Run(const GCodeFrame_t* frame) {
    if (frame == NULL) return;

    if (frame->type == GCMD_G0 || frame->type == GCMD_G1) {
        
        // 1. 解析新的目标坐标：如果有新坐标就使用新坐标，没有就保持在原地
        float target_x = frame->has_x ? frame->x : current_x;
        float target_y = frame->has_y ? frame->y : current_y;
        float target_z = frame->has_z ? frame->z : current_z;
        
        // 2. 更新运动速度 (F值)，单位 mm/min
        if (frame->has_f && frame->f > 0) {
            current_feedrate = (float)frame->f;
        }

        // 3. 计算起点到终点的三维空间直线距离 (毫米)
        float dx = target_x - current_x;
        float dy = target_y - current_y;
        float dz = target_z - current_z;
        float distance = sqrtf(dx * dx + dy * dy + dz * dz); // 勾股定理算距离

        // 4. 根据距离和速度，计算出真正的运动耗时 (毫秒)
        // 公式推导：
        // 速度 mm/ms = current_feedrate / 60000.0f
        // 耗时 ms = 距离 / (速度 mm/ms) = (距离 * 60000.0f) / current_feedrate
        uint32_t duration_ms = 0;
        
        if (distance > 0.001f) { // 只有移动距离大于 0.001mm 才计算时间，防止原地死等
            duration_ms = (uint32_t)((distance * 60000.0f) / current_feedrate);
            
            // 安全限制：防止计算出 0 毫秒导致底层除零崩溃
            if (duration_ms == 0) {
                duration_ms = 1;
            }
        }

        // 5. 将计算好的安全坐标和真实耗时，塞入底层轨迹规划器
        // 此时底层规划器收到的 duration 永远是合理的，能保证匀速画线
        Motion_Planner_MoveLine(target_x, 
                                target_y, 
                                target_z, 
                                duration_ms);

        // 6. 运动已交由底层处理，更新当前坐标记录，为下一条线段做准备
        current_x = target_x;
        current_y = target_y;
        current_z = target_z;
    }
    // else if (frame->type == GCMD_Mxx) { 处理夹爪、激光等 }
}