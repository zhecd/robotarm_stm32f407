/**
 * @file    bsp_led.h
 * @brief   Board support package for 4 user LEDs (PC0-PC3, active-low) / 4路LED板级支持 (PC0-PC3, 低电平有效)
 * @ingroup bsp
 */

#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_0 = 0,      /* LED 0 / LED 0 */
    LED_1,          /* LED 1 / LED 1 */
    LED_2,          /* LED 2 / LED 2 */
    LED_3,          /* LED 3 / LED 3 */
    LED_COUNT       /* Total LED count / LED 总数 */
} LedNumber_t;

typedef enum {
    LED_OFF = 0,    /* LED off / LED 灭 */
    LED_ON  = 1     /* LED on / LED 亮 */
} LedState_t;

void        BSP_LED_Init(void);
bool        BSP_LED_SetState(LedNumber_t led, LedState_t state);
LedState_t  BSP_LED_GetState(LedNumber_t led);
bool        BSP_LED_Toggle(LedNumber_t led);
bool        BSP_LED_SetAllStates(const LedState_t states[LED_COUNT]);
bool        BSP_LED_GetAllStates(LedState_t states[LED_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_LED_H__ */
