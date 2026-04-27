#ifndef APP_TELEOP_H
#define APP_TELEOP_H

#include "main.h"

typedef enum
{
    SYS_MODE_GCODE = 0,
    SYS_MODE_PS2
} SystemMode_t;

extern SystemMode_t current_sys_mode;

void App_Teleop_Init(void);
void App_Teleop_Task(void);

#endif /* APP_TELEOP_H */
