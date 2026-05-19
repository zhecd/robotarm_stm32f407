/**
 * @file    app_teleop.h
 * @brief   PS2 joystick teleoperation with SELECT-button mode toggle / PS2 摇杆遥控, SELECT 键切换模式
 * @ingroup app
 *
 * Toggles between SYS_MODE_GCODE and SYS_MODE_PS2 / 在 G-code 模式和 PS2 遥控模式间切换
 * In PS2 mode: left stick X/Y -> Cartesian X/Y, right stick Y -> Z,
 * Cross/Square -> gripper close/open / PS2 模式下左摇杆控制 XY, 右摇杆 Y 控制 Z, 叉/方键控制夹爪
 */

#ifndef __APP_TELEOP_H__
#define __APP_TELEOP_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SYS_MODE_GCODE = 0,     /* G-code command mode / G-code 指令模式 */
    SYS_MODE_PS2            /* PS2 teleop mode / PS2 遥控模式 */
} SystemMode_t;

SystemMode_t App_Teleop_GetMode(void);
void         App_Teleop_SetMode(SystemMode_t mode);
void         App_Teleop_Init(void);
void         App_Teleop_Task(void);       /* Call in main loop / 主循环中调用 */

#ifdef __cplusplus
}
#endif

#endif /* __APP_TELEOP_H__ */
