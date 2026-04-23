#include "app_teleop.h"
#include "bsp_ps2.h"
#include "motion_planner.h"
#include "motor_core.h"
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
        // 1. 模式安全切换逻辑
        if ((last_buttons & PS2_BTN_SELECT) && !(my_ps2.buttons & PS2_BTN_SELECT)) {
            if (Motor_Buffer_GetCount() == 0) {
                current_sys_mode = (current_sys_mode == SYS_MODE_GCODE) ? SYS_MODE_PS2 : SYS_MODE_GCODE;
                printf("\r\n>>> MODE SWITCHED TO: [%s] <<<\r\n", 
                       (current_sys_mode == SYS_MODE_GCODE) ? "G-CODE" : "PS2 TELEOP");
            } else {
                printf("Warning: Please wait for motors to stop before switching mode!\r\n");
            }
        }
        last_buttons = my_ps2.buttons;
        
        // 2. 遥控摇杆解析逻辑
        if (current_sys_mode == SYS_MODE_PS2) {
            float dx = 0.0f, dy = 0.0f, dz = 0.0f;

            int joy_ly = 128 - my_ps2.LY; 
            int joy_lx = my_ps2.LX - 128; 
            int joy_ry = 128 - my_ps2.RY; 

            // 你的自定义坐标映射
            if (abs(joy_lx) > 15) dx = (joy_lx / 128.0f) * 1.5f; 
            if (abs(joy_ly) > 15) dy = (joy_ly / 128.0f) * 1.5f; 
            if (abs(joy_ry) > 15) dz = (joy_ry / 128.0f) * 1.5f; 

            if (dx != 0.0f || dy != 0.0f || dz != 0.0f) {
                Motion_Planner_TeleopStep(dx, dy, dz);
            }
        }
    }
}