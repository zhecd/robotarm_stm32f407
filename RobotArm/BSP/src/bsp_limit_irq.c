/**
 * @file bsp_limit_irq.c
 * @brief 限位中断屏蔽端口：供诊断限流和过载停止使用，保留GPIO和其他轴中断。
 */
#include "bsp/bsp_robot_board.h"
#include "main.h"

void BSP_RobotBoard_DispatchLimitInterrupts(void)
{
    const uint16_t pins[]={M1_STOP_Pin,M2_STOP_Pin,M3_STOP_Pin};
    uint32_t pending=EXTI->PR & (M1_STOP_Pin|M2_STOP_Pin|M3_STOP_Pin);
    for(uint8_t axis=0;axis<3U;++axis){
        /* HAL处理函数只检查PR，不检查IMR。共享向量由另一轴进入时，
         * 不能再把已经屏蔽但仍pending的轴送进回调。中途屏蔽也要尊重。 */
        if(pending & pins[axis]){
            if(EXTI->IMR & pins[axis])HAL_GPIO_EXTI_IRQHandler(pins[axis]);
            else __HAL_GPIO_EXTI_CLEAR_IT(pins[axis]); /* 不回调，但不能留下共享向量的残留请求。 */
        }
    }
    /* 回调可能刚因过载屏蔽本轴；同一期间再出现的pending也必须排空。 */
    uint32_t masked=EXTI->PR & ~EXTI->IMR & (M1_STOP_Pin|M2_STOP_Pin|M3_STOP_Pin);
    if(masked)__HAL_GPIO_EXTI_CLEAR_IT(masked);
}

void BSP_RobotBoard_MaskLimitInterrupt(uint8_t axis)
{
    const uint16_t pins[]={M1_STOP_Pin,M2_STOP_Pin,M3_STOP_Pin};
    if (axis<3U){
        uint32_t pin=pins[axis];
        EXTI->IMR&=~pin;
        EXTI->RTSR&=~pin;EXTI->FTSR&=~pin;
        __HAL_GPIO_EXTI_CLEAR_IT(pin);
    }
}

void BSP_RobotBoard_ResetLimitInterrupts(void)
{
    uint32_t pins=M1_STOP_Pin|M2_STOP_Pin|M3_STOP_Pin;
#if ROBOTARM_LIMIT_STATE_MODE
    /* 状态模式的恢复仅重建软件状态，不得悄悄重新开启限位边沿检测。 */
    EXTI->IMR&=~pins;EXTI->EMR&=~pins;
    EXTI->RTSR&=~pins;EXTI->FTSR&=~pins;
    __HAL_GPIO_EXTI_CLEAR_IT(pins);
#else
    /* PR按位写1清除，不能用|=，避免顺带清掉其他设备的pending。 */
    EXTI->RTSR&=~pins;EXTI->FTSR|=pins; /* 本板限位恢复为原下降沿检测。 */
    __HAL_GPIO_EXTI_CLEAR_IT(pins);
    EXTI->IMR|=pins;
#endif
}
bool BSP_RobotBoard_TryRearmLimitInterrupt(uint8_t axis)
{
    const uint16_t pins[]={M1_STOP_Pin,M2_STOP_Pin,M3_STOP_Pin};
    if(axis>=3U || BSP_RobotBoard_IsLimitTriggered(axis))return false;
    uint32_t pin=pins[axis];
#if ROBOTARM_LIMIT_STATE_MODE
    /* 沿用上层“释放后重新监视”的握手，确认的是GPIO状态；IRQ保持关闭。
     * 调用者已确认连续松开10ms，此处再次读回，避免交接时又压住。 */
    EXTI->IMR&=~pin;EXTI->EMR&=~pin;
    EXTI->RTSR&=~pin;EXTI->FTSR&=~pin;
    __HAL_GPIO_EXTI_CLEAR_IT(pin);
    return !BSP_RobotBoard_IsLimitTriggered(axis);
#else
    EXTI->RTSR&=~pin;EXTI->FTSR|=pin;
    __HAL_GPIO_EXTI_CLEAR_IT(pin); /* 精确W1C；不清其他轴/按键pending。 */
    EXTI->IMR|=pin;
    /* 恢复窗口内若已重新压住，立即收回恢复许可，不把它当作已释放。 */
    if(BSP_RobotBoard_IsLimitTriggered(axis)){
        BSP_RobotBoard_MaskLimitInterrupt(axis);return false;
    }
    return true;
#endif
}
