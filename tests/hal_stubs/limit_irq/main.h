#ifndef TEST_LIMIT_IRQ_MAIN_H
#define TEST_LIMIT_IRQ_MAIN_H
#include <stdint.h>
typedef struct { volatile uint32_t IMR,PR,RTSR,FTSR,EMR; } EXTI_TypeDef;
extern EXTI_TypeDef test_exti;
#define EXTI (&test_exti)
#define M1_STOP_Pin (1U<<10)
#define M2_STOP_Pin (1U<<11)
#define M3_STOP_Pin (1U<<12)
void HAL_GPIO_EXTI_IRQHandler(uint16_t pin);
void Test_ClearPending(uint32_t pin);
#define __HAL_GPIO_EXTI_CLEAR_IT(pin) Test_ClearPending(pin)
#define __HAL_GPIO_EXTI_GET_IT(pin) (EXTI->PR & (pin))
#define RESET 0U
void HAL_GPIO_EXTI_Callback(uint16_t pin);
#endif
