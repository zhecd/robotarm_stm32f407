#ifndef ROBOTARM_APP_HARDWARE_ADAPTER_H
#define ROBOTARM_APP_HARDWARE_ADAPTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t left_x;
    uint8_t left_y;
    uint8_t right_y;
    bool cross_pressed;
    bool square_pressed;
} AppTeleopInput_t;

void AppHardware_Init(void);
void AppHardware_ConfigureMotionDrivers(void);
void AppHardware_EnableJoints(bool enabled);
void AppHardware_SetModeIndicator(bool gcode_on, bool ps2_on);
void AppHardware_StartStepTimer(void);

bool AppHardware_IsLimitTriggered(uint8_t joint_index);
bool AppHardware_ReadTeleopInput(AppTeleopInput_t *out_input);

void AppHardware_SendText(const char *text);
bool AppHardware_TakeLineTimeout(void);
bool AppHardware_TakeRxOverflow(void);
bool AppHardware_TakeTxOverflow(void);
bool AppHardware_ReadLine(char *out_line, size_t out_size);
void AppHardware_DiscardRx(void);

#endif /* ROBOTARM_APP_HARDWARE_ADAPTER_H */
