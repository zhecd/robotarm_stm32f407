/** @file dev_input.h @brief Operator-input device abstraction. */
#ifndef ROBOTARM_DEV_INPUT_H
#define ROBOTARM_DEV_INPUT_H

#include <stdbool.h>
#include <stdint.h>

/* Active-low button state: 0 means pressed. */
#define DEV_INPUT_BTN_CROSS  0x0040U
#define DEV_INPUT_BTN_SQUARE 0x0080U

typedef struct {
    uint16_t buttons;
    uint8_t  left_x;
    uint8_t  left_y;
    uint8_t  right_x;
    uint8_t  right_y;
} DevInputState_t;

void Dev_Input_Init(void);
bool Dev_Input_Read(DevInputState_t *state);
bool Dev_Input_IsAnalogMode(void);

#endif /* ROBOTARM_DEV_INPUT_H */
