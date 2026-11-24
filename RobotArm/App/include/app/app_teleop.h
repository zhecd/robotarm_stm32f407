/**
 * @file app_teleop.h
 * @brief PS2 遥操作：摇杆死区、增量命令和夹爪按键边沿处理。
 */

#ifndef __APP_TELEOP_H__
#define __APP_TELEOP_H__


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SYS_MODE_GCODE = 0,     /* G-code 串口命令模式。 */
    SYS_MODE_PS2            /* PS2 手柄遥操作模式。 */
} SystemMode_t;

SystemMode_t App_Teleop_GetMode(void);
void         App_Teleop_SetMode(SystemMode_t mode); /* 切换后重新等待摇杆回中、夹爪键松开。 */
void         App_Teleop_ToggleMode(void);     /* 由板端MODE事件调用，串口与PS2互斥。 */
void         App_Teleop_Init(void);           /* 启动/重新回零后重置输入接管状态。 */
void         App_Teleop_Task(void);           /* 主循环调用；G-code模式不轮询手柄。 */

#ifdef __cplusplus
}
#endif

#endif /* __APP_TELEOP_H__ */
