/**
 * @file bsp_led.h
 * @brief 板级 LED：逻辑编号与实际 GPIO、电平的映射。
 */

#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_0 = 0,      /* LED 0 */
    LED_1,          /* LED 1 */
    LED_2,          /* LED 2 */
    LED_3,          /* LED 3 */
    LED_COUNT       /* Total LED count */
} LedNumber_t;

typedef enum {
    LED_OFF = 0,    /* LED 熄灭。 */
    LED_ON  = 1     /* LED 点亮。 */
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
