#ifndef __APP_TELEOP_H
#define __APP_TELEOP_H

#include "main.h"
#include <stdbool.h>

// 将系统模式枚举移动到这里（它属于应用层的状态）
typedef enum {
    SYS_MODE_GCODE,  // 写字机模式
    SYS_MODE_PS2     // 手柄遥控模式
} SystemMode_t;

// 暴露当前模式，方便其他模块（如 G 代码模块）查询
extern SystemMode_t current_sys_mode;

// 应用层对外提供的标准接口
void App_Teleop_Init(void);
void App_Teleop_Task(void);

#endif