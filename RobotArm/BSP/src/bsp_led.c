/**
 * @file    bsp_led.c
 * @brief   LED board support package implementation / LED 鏉跨骇鏀寔鍖呭疄鐜?
 * @ingroup bsp
 */

#include "bsp/bsp_led.h"

/* LED software state array, initialized to all-off / LED 杞欢鐘舵€佹暟�? 鍒濆鍏ㄧ伃 */
static LedState_t        s_led_states[LED_COUNT] = {LED_OFF, LED_OFF, LED_OFF, LED_OFF};
static const uint16_t    s_led_pins[LED_COUNT]   = {LED0_Pin, LED1_Pin, LED2_Pin, LED3_Pin};
static GPIO_TypeDef     *const s_led_ports[LED_COUNT] = {LED0_GPIO_Port, LED1_GPIO_Port, LED2_GPIO_Port, LED3_GPIO_Port};

void BSP_LED_Init(void)
{
    for (uint8_t i = 0U; i < LED_COUNT; i++) {
        s_led_states[i] = LED_OFF;
        HAL_GPIO_WritePin(s_led_ports[i], s_led_pins[i], GPIO_PIN_SET);
    }
}

bool BSP_LED_SetState(LedNumber_t led, LedState_t state)
{
    if (led >= LED_COUNT) return false;
    s_led_states[led] = state;
    HAL_GPIO_WritePin(s_led_ports[led], s_led_pins[led],
                      (state == LED_ON) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    return true;
}

LedState_t BSP_LED_GetState(LedNumber_t led)
{
    return (led < LED_COUNT) ? s_led_states[led] : LED_OFF;
}

bool BSP_LED_Toggle(LedNumber_t led)
{
    if (led >= LED_COUNT) return false;
    return BSP_LED_SetState(led,
        (s_led_states[led] == LED_ON) ? LED_OFF : LED_ON);
}

bool BSP_LED_SetAllStates(const LedState_t states[LED_COUNT])
{
    if (!states) return false;
    for (int i = 0; i < LED_COUNT; i++) {
        s_led_states[i] = states[i];
        HAL_GPIO_WritePin(s_led_ports[i], s_led_pins[i],
                          (states[i] == LED_ON) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    return true;
}

bool BSP_LED_GetAllStates(LedState_t states[LED_COUNT])
{
    if (!states) return false;
    for (int i = 0; i < LED_COUNT; i++) {
        states[i] = s_led_states[i];
    }
    return true;
}
