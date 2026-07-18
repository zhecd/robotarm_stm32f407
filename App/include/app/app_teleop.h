/**
 * @file    app_teleop.h
 * @brief   PS2 joystick teleoperation with SELECT-button mode toggle / PS2 鎽囨潌閬ユ帶, SELECT 閿垏鎹㈡ā寮?
 * @ingroup app
 *
 * Toggles between SYS_MODE_GCODE and SYS_MODE_PS2 / �?G-code 妯″紡�?PS2 閬ユ帶妯″紡闂村垏鎹?
 * In PS2 mode: left stick X/Y -> Cartesian X/Y, right stick Y -> Z,
 * Cross/Square -> gripper close/open / PS2 妯″紡涓嬪乏鎽囨潌鎺у埗 XY, 鍙虫憞鏉?Y 鎺у埗 Z, �?鏂归敭鎺у埗澶圭埅
 */

#ifndef __APP_TELEOP_H__
#define __APP_TELEOP_H__


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SYS_MODE_GCODE = 0,     /* G-code command mode / G-code 鎸囦护妯″紡 */
    SYS_MODE_PS2            /* PS2 teleop mode / PS2 閬ユ帶妯″紡 */
} SystemMode_t;

SystemMode_t App_Teleop_GetMode(void);
void         App_Teleop_SetMode(SystemMode_t mode);
void         App_Teleop_ToggleMode(void);     /* Toggle between GCode <-> PS2 / �?GCode �?PS2 涔嬮棿鍒囨崲 */
void         App_Teleop_Init(void);
void         App_Teleop_Task(void);           /* Call in main loop / 涓诲惊鐜腑璋冪�?*/

#ifdef __cplusplus
}
#endif

#endif /* __APP_TELEOP_H__ */
