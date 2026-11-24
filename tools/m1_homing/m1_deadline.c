/** @file m1_deadline.c @brief 同时处理TIM5外围标志和NVIC pending两级状态。 */
#include "m1_deadline.h"
#include "m1_logic.h"
#include "main.h"
bool M1Deadline_Prepare(uint32_t ms)
{
    /* UG会装载预分频/重装值，也会产生更新事件。先关UIE，再生成UG；
     * 即使随后清了SR，已经锁存在NVIC的pending也必须另行清除。
     * 初版UIE开着生成UG、ISR又不检查UIF，导致CNT=0就误报截止。 */
    TIM5->DIER=0;TIM5->CR1=0;
    if(ms==0U || ms>M1_TIMER_MAX_MS){
        TIM5->SR=0;HAL_NVIC_ClearPendingIRQ(TIM5_IRQn);return false;
    }
    TIM5->PSC=8399U;TIM5->ARR=ms*10U-1U;TIM5->CNT=0;
    TIM5->EGR=TIM_EGR_UG;TIM5->SR=0;__DSB();
    HAL_NVIC_ClearPendingIRQ(TIM5_IRQn);
    TIM5->DIER=TIM_DIER_UIE;
    return true;
}
bool M1Deadline_ExpiredFromISR(void)
{
    if(!(TIM5->SR&TIM_SR_UIF) || !(TIM5->DIER&TIM_DIER_UIE))return false;
    TIM5->DIER=0;TIM5->SR=0;TIM5->CR1=0;return true;
}
