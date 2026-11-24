/** 真实BSP+原厂HAL处理函数+真实过载计数；寄存器W1C和异步到达仅由端口模型替换。
 * 不模拟真实NVIC/APB时序。注入的pending是测试刺激，不能冒称实物自发中断。
 */
#include "main.h"
#include "bsp/bsp_robot_board.h"
#include "limit_irq_guard.h"
#include "platform_critical.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
EXTI_TypeDef test_exti;
static uint32_t now,depth,calls[3],reassert_pin,reassert_left,clear_writes;
bool BSP_RobotBoard_IsLimitTriggered(uint8_t axis){(void)axis;return false;}
uint32_t PlatformTime_NowMs(void){return now;}
PlatformCriticalState_t PlatformCritical_Enter(void){return depth++;}
void PlatformCritical_Exit(PlatformCriticalState_t old){assert(depth==old+1);depth=old;}
void Test_ClearPending(uint32_t pin)
{
    assert((pin&~0x1c00U)==0U);clear_writes++;
    test_exti.PR&=~pin; /* MMIO端口模拟真实按位写1清除。 */
    if(reassert_left && (pin&reassert_pin) && (test_exti.FTSR&reassert_pin)) {
        --reassert_left;test_exti.PR|=reassert_pin; /* 在清除后明确注入下一次事件。 */
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    unsigned axis=pin==M1_STOP_Pin?0U:pin==M2_STOP_Pin?1U:2U;
    assert(pin==(1U<<(10U+axis)));calls[axis]++;
    if(LimitIrqGuard_OnEdgeFromISR((uint8_t)axis))BSP_RobotBoard_MaskLimitInterrupt((uint8_t)axis);
}
static void ResetRegisters(void)
{
    test_exti=(EXTI_TypeDef){.IMR=0x1c04,.PR=4,.FTSR=0x1c04};
    calls[0]=calls[1]=calls[2]=0;reassert_pin=reassert_left=clear_writes=0;now+=100;
}
int main(void)
{
    for(unsigned enabled=0;enabled<8;enabled++)for(unsigned pending=0;pending<8;pending++) {
        ResetRegisters();test_exti.IMR=(enabled<<10)|4U;test_exti.PR=(pending<<10)|4U;
        BSP_RobotBoard_DispatchLimitInterrupts();
        for(unsigned a=0;a<3;a++)assert(calls[a]==(((enabled&pending)>>a)&1U));
        assert(test_exti.PR==4U && LimitIrqGuard_GetLatchMask()==0);
    }
    /* 一次pending应只产生一次回调；反复调用空分发不能凭空增加计数。 */
    ResetRegisters();test_exti.PR|=M3_STOP_Pin;
    BSP_RobotBoard_DispatchLimitInterrupts();assert(calls[2]==1 && clear_writes==1);
    for(unsigned i=0;i<300;i++)BSP_RobotBoard_DispatchLimitInterrupts();
    assert(calls[2]==1 && test_exti.PR==4 && LimitIrqGuard_GetLatchMask()==0);
    /* 只有显式重新置pending才会继续回调；256次后真实过载服务屏蔽该轴。 */
    ResetRegisters();reassert_pin=M3_STOP_Pin;reassert_left=300;test_exti.PR|=M3_STOP_Pin;
    unsigned vectors=0;
    while((test_exti.PR&test_exti.IMR&0x1c00U) && vectors<400){
        BSP_RobotBoard_DispatchLimitInterrupts();vectors++;
    }
    assert(vectors==256 && calls[2]==256 && !calls[0] && !calls[1]);
    assert(LimitIrqGuard_GetLatchMask()==4 && !(test_exti.IMR&M3_STOP_Pin));
    assert(!(test_exti.FTSR&M3_STOP_Pin) && test_exti.PR==4U);
    assert(test_exti.IMR&(M1_STOP_Pin|M2_STOP_Pin));
    puts("Original HAL + BSP: 64 mask/pending combinations; no spontaneous callbacks; explicit reassertion and real overload latch passed");
    return 0;
}
