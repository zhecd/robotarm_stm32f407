#include "motion_planner.h"
#include "robotGeometry.h" 
#include "motor_core.h"    
#include "bsp_stepper.h"   
#include <stdlib.h>        
#include <math.h>          

#define TICKS_PER_MS  50
#define LINEAR_STEP_MM 1.0f 

// ==============================================================
// 🌟 动力学魔法：五次多项式平滑函数 (Ken Perlin's Smootherstep)
// ==============================================================
// 输入 u: 局部进度 (0.0 到 1.0)
// 输出: 速度倍率 (0.0 到 1.0)，保证一阶和二阶导数在两端全为 0
static float Quintic_Smoothstep(float u) 
{
    if (u <= 0.0f) return 0.0f;
    if (u >= 1.0f) return 1.0f;
    float u3 = u * u * u;
    float u4 = u3 * u;
    float u5 = u4 * u;
    return 6.0f * u5 - 15.0f * u4 + 10.0f * u3;
}

// 静态记忆变量
static int32_t planned_pos_m1 = 0;
static int32_t planned_pos_m2 = 0;
static int32_t planned_pos_m3 = 0;
static float current_x = 0.0f;
static float current_y = 0.0f;
static float current_z = 0.0f;

void Motion_Planner_Init(float start_x, float start_y, float start_z)
{
    current_x = start_x;
    current_y = start_y;
    current_z = start_z;
    planned_pos_m1 = Motor_M1.absolute_position;
    planned_pos_m2 = Motor_M2.absolute_position;
    planned_pos_m3 = Motor_M3.absolute_position;
}

bool Motion_Planner_MoveLine(float target_x, float target_y, float target_z, uint32_t duration_ms) 
{
    float dx = target_x - current_x;
    float dy = target_y - current_y;
    float dz = target_z - current_z;
    float distance = sqrtf(dx*dx + dy*dy + dz*dz);

    if (distance < 0.1f) return true;

    uint32_t segments = (uint32_t)(distance / LINEAR_STEP_MM);
    if (segments == 0) segments = 1; 

    // 这个是“巡航阶段”的基础速度 (全速时的 Ticks)
    uint32_t base_ticks = (duration_ms * TICKS_PER_MS) / segments;
    if (base_ticks == 0) base_ticks = 1;

    // ==============================================================
    // 🌟 轨迹规划参数设置
    // ==============================================================
    // 我们将一条直线分为 3 个阶段：加速区(20%)、巡航区(60%)、减速区(20%)
    const float ACCEL_ZONE = 0.2f; 
    const float DECEL_ZONE = 0.2f; 
    // 最低速度限制 (设为 15% 速度，防止起步速度为 0 导致 ticks 变无穷大卡死)
    const float V_MIN_RATIO = 0.15f; 

    for (uint32_t i = 1; i <= segments; i++) 
    {
        // 1. 空间插补：计算当前微小目标点的 (x, y, z)
        float progress = (float)i / (float)segments; // 总体进度 0.0 到 1.0
        float step_x = current_x + dx * progress;
        float step_y = current_y + dy * progress;
        float step_z = current_z + dz * progress;

        // 2. ★ 核心：五次多项式 S型速度曲线分配 ★
        float v_mult = 1.0f; // 默认速度倍率是 1.0 (全速)

        if (progress < ACCEL_ZONE) 
        {
            // 在加速区：将局部进度映射到 0.0 ~ 1.0，喂给五次多项式
            float u = progress / ACCEL_ZONE; 
            v_mult = V_MIN_RATIO + (1.0f - V_MIN_RATIO) * Quintic_Smoothstep(u);
        } 
        else if (progress > (1.0f - DECEL_ZONE)) 
        {
            // 在减速区：将局部进度映射到 0.0 ~ 1.0，喂给五次多项式然后反转
            float u = (progress - (1.0f - DECEL_ZONE)) / DECEL_ZONE;
            v_mult = 1.0f - (1.0f - V_MIN_RATIO) * Quintic_Smoothstep(u);
        }

        // 3. 时间膨胀魔法：速度越慢 (v_mult越小)，分配的 Ticks 就越多！
        uint32_t current_ticks = (uint32_t)(base_ticks / v_mult);

        // 4. 逆运动学解算
        RobotAngles target_angles;
        RobotMotorUnits target_units;
        MotionFrame_t frame;

        RobotGeometry_CalculateAngles(step_x, step_y, step_z, &target_angles);
        RobotGeometry_AnglesToMotorUnits(&target_angles, &target_units);

        frame.delta_m1 = target_units.rotUnits - planned_pos_m1;
        frame.delta_m2 = target_units.lowUnits - planned_pos_m2;
        frame.delta_m3 = target_units.highUnits - planned_pos_m3;
        frame.total_ticks = current_ticks; // 赋予带有 S 型魔法的时间参数！

        // 5. 超速安全硬件锁
        uint32_t max_delta = abs(frame.delta_m1);
        if (abs(frame.delta_m2) > max_delta) max_delta = abs(frame.delta_m2);
        if (abs(frame.delta_m3) > max_delta) max_delta = abs(frame.delta_m3);
        if (max_delta > frame.total_ticks) {
            frame.total_ticks = max_delta + 5; 
        }

        // 6. 阻塞式压入
        while (!Motor_Buffer_Push(&frame)) {}

        planned_pos_m1 = target_units.rotUnits;
        planned_pos_m2 = target_units.lowUnits;
        planned_pos_m3 = target_units.highUnits;
    }

    current_x = target_x;
    current_y = target_y;
    current_z = target_z;

    return true;
}

// [新增] 遥控专用接口：无阻塞、无 S 曲线、低水位饥饿喂食
// ==============================================================
bool Motion_Planner_TeleopStep(float dx, float dy, float dz) 
{
    // ★ 核心魔法：队列低水位检测 ★
    // 如果底层已经有 >= 2 个动作在排队了，直接丢弃新摇杆指令！
    // 这保证了手柄遥控时的延迟永远小于 40ms，松手立刻刹车！
    if (Motor_Buffer_GetCount() >= 2) {
        return false; 
    }

    float target_x = current_x + dx;
    float target_y = current_y + dy;
    float target_z = current_z + dz;

    RobotAngles target_angles;
    RobotMotorUnits target_units;
    MotionFrame_t frame;

    // 逆运动学解算：计算这个微小目标点需要各个电机转到什么位置
    RobotGeometry_CalculateAngles(target_x, target_y, target_z, &target_angles);
    RobotGeometry_AnglesToMotorUnits(&target_angles, &target_units);

    // 计算电机增量步数
    frame.delta_m1 = target_units.rotUnits - planned_pos_m1;
    frame.delta_m2 = target_units.lowUnits - planned_pos_m2;
    frame.delta_m3 = target_units.highUnits - planned_pos_m3;
    
    // 遥控模式下赋予一个极短的固定执行时间 (20ms)
    // TICKS_PER_MS 在此文件顶部定义为 50
    frame.total_ticks = 20 * 50; 

    // 非阻塞压入队列
    if (Motor_Buffer_Push(&frame)) {
        // 如果成功压入队列，更新内部的静态坐标记忆
        planned_pos_m1 = target_units.rotUnits;
        planned_pos_m2 = target_units.lowUnits;
        planned_pos_m3 = target_units.highUnits;
        
        current_x = target_x;
        current_y = target_y;
        current_z = target_z;
        return true;
    }
    
    return false;
}