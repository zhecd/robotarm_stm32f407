#include "app_teleop.h"
#include "bsp_ps2.h"
#include "motion_planner.h"
#include "motor_core.h"
#include "bsp_gripper.h" // ★ 新增：引入夹爪底层驱动
#include <stdio.h>
#include <stdlib.h>

// 全局系统模式变量
SystemMode_t current_sys_mode = SYS_MODE_GCODE;

// 本模块私有变量 (对外隐藏)
static uint32_t last_ps2_read_time = 0;
static uint16_t last_buttons = 0xFFFF;

void App_Teleop_Init(void) {
    last_ps2_read_time = HAL_GetTick();
    printf("App_Teleop initialized.\r\n");
}

// 遥控任务轮询函数（非阻塞）
void App_Teleop_Task(void) {
    // 严格限制 50Hz (20ms) 的轮询频率
    if (HAL_GetTick() - last_ps2_read_time < 20) {
        return; 
    }
    last_ps2_read_time = HAL_GetTick();

    PS2_Data_t my_ps2 = {0}; 
    bool is_ps2_connected = BSP_PS2_ReadData(&my_ps2);

    if (is_ps2_connected) {
        // 1. 模式安全切换逻辑 (SELECT 键)
        if ((last_buttons & PS2_BTN_SELECT) && !(my_ps2.buttons & PS2_BTN_SELECT)) {
            if (Motor_Buffer_GetCount() == 0) {
                current_sys_mode = (current_sys_mode == SYS_MODE_GCODE) ? SYS_MODE_PS2 : SYS_MODE_GCODE;
                printf("\r\n>>> MODE SWITCHED TO: [%s] <<<\r\n", 
                       (current_sys_mode == SYS_MODE_GCODE) ? "G-CODE" : "PS2 TELEOP");
            } else {
                printf("Warning: Please wait for motors to stop before switching mode!\r\n");
            }
        }
        
        // 2. 遥控摇杆与按键解析逻辑 (仅在 PS2 模式下有效)
        if (current_sys_mode == SYS_MODE_PS2) {
            
            // --- 摇杆解析 (控制空间位置) ---
            float dx = 0.0f, dy = 0.0f, dz = 0.0f;

            int joy_ly = 128 - my_ps2.LY; 
            int joy_lx = my_ps2.LX - 128; 
            int joy_ry = 128 - my_ps2.RY; 

            // 自定义坐标映射与死区消除
            if (abs(joy_lx) > 15) dx = (joy_lx / 128.0f) * 1.5f; 
            if (abs(joy_ly) > 15) dy = (joy_ly / 128.0f) * 1.5f; 
            if (abs(joy_ry) > 15) dz = (joy_ry / 128.0f) * 1.5f; 

            if (dx != 0.0f || dy != 0.0f || dz != 0.0f) {
                Motion_Planner_TeleopStep(dx, dy, dz);
            }

            // --- ★ 新增：按键解析 (控制夹爪) ★ ---
            // 检测 X 键 (叉号) 被按下的瞬间 -> 闭合夹爪 (抓取)
            if ((last_buttons & PS2_BTN_CROSS) && !(my_ps2.buttons & PS2_BTN_CROSS)) {
                BSP_Gripper_Close(&hgripper);
                // 打印提示，方便你在串口助手里确认触发状态
                printf("PS2: Gripper Close (X Pressed)\r\n"); 
            }
            
            // 检测 方形键 被按下的瞬间 -> 张开夹爪 (释放)
            if ((last_buttons & PS2_BTN_SQUARE) && !(my_ps2.buttons & PS2_BTN_SQUARE)) {
                BSP_Gripper_Open(&hgripper);
                printf("PS2: Gripper Open (Square Pressed)\r\n");
            }
        }
        
        // 最后更新按键状态，用于下一次的边沿检测
        last_buttons = my_ps2.buttons;
    }
}