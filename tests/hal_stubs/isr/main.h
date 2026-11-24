/* 仅替换 STM32 ISR 参数类型；被测文件仍为真实 app_isr_adapter.c。 */
#ifndef TEST_ISR_MAIN_H
#define TEST_ISR_MAIN_H
#include <stdint.h>
typedef struct { uintptr_t Instance; } TIM_HandleTypeDef;
#define TIM6 ((uintptr_t)6U)
#define M1_STOP_Pin ((uint16_t)(1U<<10))
#define M2_STOP_Pin ((uint16_t)(1U<<11))
#define M3_STOP_Pin ((uint16_t)(1U<<12))
#define KEY_MODE_Pin ((uint16_t)(1U<<2))
#endif
