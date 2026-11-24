/* BSP停止测试的HAL类型/函数端口，不模拟物理时序。 */
#ifndef TEST_STOP_MAIN_H
#define TEST_STOP_MAIN_H
#include <stdint.h>
typedef struct { uint32_t value; } GPIO_TypeDef;
typedef struct { uintptr_t Instance; } TIM_HandleTypeDef;
typedef struct { unsigned value; } UART_HandleTypeDef;
typedef enum { GPIO_PIN_RESET,GPIO_PIN_SET } GPIO_PinState;
#define TIM6 ((uintptr_t)6U)
#define TIM2 ((uintptr_t)2U)
#define TIM_CHANNEL_2 4U
#define HAL_OK 0
#define M1_STOP_GPIO_Port ((GPIO_TypeDef *)0)
#define M2_STOP_GPIO_Port ((GPIO_TypeDef *)0)
#define M3_STOP_GPIO_Port ((GPIO_TypeDef *)0)
#define M1_STOP_Pin 1U
#define M2_STOP_Pin 2U
#define M3_STOP_Pin 4U
int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer);
int HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *timer);
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer,uint32_t channel);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port,uint16_t pin);
#endif
