#ifndef TEST_M1_DEADLINE_MAIN_H
#define TEST_M1_DEADLINE_MAIN_H
#include <stdint.h>
typedef struct {uint32_t CR1,DIER,SR,PSC,ARR,CNT,EGR;} TestDeadlineTimer;
extern TestDeadlineTimer deadline_timer;
#define TIM5 (&deadline_timer)
#define TIM5_IRQn 50
#define TIM_EGR_UG 1U
#define TIM_DIER_UIE 1U
#define TIM_SR_UIF 1U
void HAL_NVIC_ClearPendingIRQ(int irq);
void DeadlineTest_Barrier(void);
#define __DSB() DeadlineTest_Barrier()
#endif
