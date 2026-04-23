#include "app_teleop.h"
#include "bsp_ps2.h"
#include "motion_planner.h"
#include "motor_core.h"
#include <stdio.h>
#include <stdlib.h>

// 全局系统模式变量
SystemMode_t current_sys_mode = SYS_MODE_GCODE;

// 本模块私有变量 (对外部文件隐藏)
static uint32_t last_ps2_read_time = 0;
static uint16_t last_buttons = 0xFFFF;

void App_Teleop_Init(void) {
    last_ps2_read_time = HAL_GetTick();
    printf("App_Teleop initialized.\r\n");
}

// 遥控任务轮询函数（非阻塞）
void App_Teleop_Task(void) {
    // 严格限制 50Hz (20ms) 的轮询频率，防止刷死接收器
    if (HAL_GetTick() - last_ps2_read_time < 20) {
        return; 
    }
    last_ps2_read_time = HAL_GetTick();

    PS2_Data_t my_ps2 = {0}; 
    
    // 【终极防御】关中断读取！防止 TIM6 步进电机定时器打断软件 SPI 的微秒级时序
    __disable_irq(); 
    bool is_ps2_connected = BSP_PS2_ReadData(&my_ps2);
    __enable_irq();  

    if (is_ps2_connected) {
        // 1. 检测 SELECT 键的"单击"动作，实现安全模式切换
        if ((last_buttons & PS2_BTN_SELECT) && !(my_ps2.buttons & PS2_BTN_SELECT)) {
            if (Motor_Buffer_GetCount() == 0) { // 必须等机械臂停稳再切模式
                current_sys_mode = (current_sys_mode == SYS_MODE_GCODE) ? SYS_MODE_PS2 : SYS_MODE_GCODE;
                printf("\r\n>>> MODE SWITCHED TO: [%s] <<<\r\n", 
                       (current_sys_mode == SYS_MODE_GCODE) ? "G-CODE" : "PS2 TELEOP");
            } else {
                printf("Warning: Please wait for motors to stop before switching mode!\r\n");
            }
        }
        last_buttons = my_ps2.buttons;
        
        // 2. 手柄遥控摇杆解析与移动控制
        if (current_sys_mode == SYS_MODE_PS2) {
            float dx = 0.0f, dy = 0.0f, dz = 0.0f;

            // 读取摇杆原始数据并转为 -128 到 +127 的有符号整数
            // (注：物理摇杆向上推Y变小，所以用 128 - LY 修正为上正下负)
            int joy_ly = 128 - my_ps2.LY; 
            int joy_lx = my_ps2.LX - 128; 
            int joy_ry = 128 - my_ps2.RY; 

            // 坐标映射逻辑 (根据你的要求定制)
            // 左摇杆左右 -> X轴 (左减, 右加)
            if (abs(joy_lx) > 15) {
                dx = (joy_lx / 128.0f) * 1.5f; 
            }
            // 左摇杆上下 -> Y轴 (下减, 上加)
            if (abs(joy_ly) > 15) {
                dy = (joy_ly / 128.0f) * 1.5f; 
            }
            // 右摇杆上下 -> Z轴 (下减, 上加)
            if (abs(joy_ry) > 15) {
                dz = (joy_ry / 128.0f) * 1.5f; 
            }

            // 如果有任何方向轴产生了位移，喂给底层运动规划器
            if (dx != 0.0f || dy != 0.0f || dz != 0.0f) {
                Motion_Planner_TeleopStep(dx, dy, dz);
            }
        }
    }
}