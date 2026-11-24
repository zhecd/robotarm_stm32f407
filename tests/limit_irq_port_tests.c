/* 编译真实Device/BSP；端口替身显式模拟W1C，检查掩码、精确清除位和分发。
 * 不模拟真实异步边沿或NVIC电平行为，后两者需独立实机对照。 */
#include "main.h"
#include "device/dev_limit_switch.h"
#include "bsp/bsp_robot_board.h"
#include <assert.h>
#include <stdio.h>
EXTI_TypeDef test_exti;
static unsigned calls[3];
static bool mask_second_during_callback;
static uint32_t last_w1c;
static uint8_t trigger_mask=2;
static unsigned read_count;
static bool press_during_rearm;
void Test_ClearPending(uint32_t pin){last_w1c=pin;test_exti.PR&=~pin;}
void HAL_GPIO_EXTI_IRQHandler(uint16_t pin)
{
    /* 模拟供应商HAL的PR判断和硬件W1C效果，故意不在替身里过滤IMR。 */
    if(test_exti.PR&pin){
        unsigned a=pin==M1_STOP_Pin?0U:pin==M2_STOP_Pin?1U:2U;
        calls[a]++;test_exti.PR&=~(uint32_t)pin;
        if(mask_second_during_callback && a==0)test_exti.IMR&=~M2_STOP_Pin;
    }
}
bool BSP_RobotBoard_IsLimitTriggered(uint8_t axis)
{return ((trigger_mask&(1U<<axis))!=0) || (press_during_rearm && ++read_count==2U);}
int main(void)
{
    const uint32_t limits=M1_STOP_Pin|M2_STOP_Pin|M3_STOP_Pin;
    for(unsigned axis=0;axis<3;axis++) {
        test_exti.IMR=limits|4U;test_exti.PR=4U;
        test_exti.RTSR=test_exti.FTSR=limits|4U;
        Dev_LimitSwitch_MaskDiagnosticInterrupt((DevJointId_t)axis);
        assert(test_exti.IMR==((limits|4U)&~(1U<<(10+axis))));
        assert(test_exti.PR==4U); /* 单轴屏蔽不能顺带清无关pending。 */
        assert(last_w1c==(1U<<(10+axis)));
        assert(test_exti.FTSR==((limits|4U)&~(1U<<(10+axis))));
        assert(test_exti.RTSR==test_exti.FTSR);
        assert(Dev_LimitSwitch_IsTriggered(DEV_JOINT_M2));
    }
    uint32_t original=test_exti.IMR;
    Dev_LimitSwitch_MaskDiagnosticInterrupt((DevJointId_t)255);
    BSP_RobotBoard_MaskLimitInterrupt(255);assert(test_exti.IMR==original);
    Dev_LimitSwitch_ResetDiagnosticInterrupts();
    assert(test_exti.IMR==(limits|4U));
    assert(last_w1c==limits && test_exti.PR==4U); /* 检查精确W1C位图与硬件语义替身。 */
    assert((test_exti.FTSR&limits)==limits && (test_exti.RTSR&limits)==0);
    assert(!Dev_LimitSwitch_IsTriggered((DevJointId_t)255));
    for(unsigned enabled=0;enabled<8;enabled++)for(unsigned pending=0;pending<8;pending++){
        test_exti.IMR=(enabled<<10)|4U;test_exti.PR=(pending<<10)|4U;
        calls[0]=calls[1]=calls[2]=0;BSP_RobotBoard_DispatchLimitInterrupts();
        for(unsigned a=0;a<3;a++)assert(calls[a]==(((enabled&pending)>>a)&1U));
        assert(test_exti.PR==4U); /* 屏蔽位排空但不回调，无关位保留。 */
    }
    test_exti.IMR=test_exti.PR=limits;calls[0]=calls[1]=calls[2]=0;
    mask_second_during_callback=true;BSP_RobotBoard_DispatchLimitInterrupts();
    assert(calls[0]==1 && calls[1]==0 && calls[2]==1 && test_exti.PR==0);
    /* 真实单轴恢复：只允许读高的轴，精确W1C，保留其他轴与按键。 */
    for(unsigned axis=0;axis<3;axis++){
        trigger_mask=0;test_exti=(EXTI_TypeDef){.IMR=4,.PR=limits|4U,.RTSR=limits|4U,.FTSR=4};
        assert(Dev_LimitSwitch_TryRearmInterrupt((DevJointId_t)axis));
        assert(test_exti.IMR==(4U|(1U<<(axis+10))));
        assert(test_exti.PR==((limits|4U)&~(1U<<(axis+10))));
        assert(last_w1c==(1U<<(axis+10)) && !(test_exti.RTSR&(1U<<(axis+10))));
        assert(test_exti.FTSR==(4U|(1U<<(axis+10))));
    }
    trigger_mask=2;test_exti.IMR=4;assert(!Dev_LimitSwitch_TryRearmInterrupt(DEV_JOINT_M2));
    assert(test_exti.IMR==4 && !Dev_LimitSwitch_TryRearmInterrupt((DevJointId_t)255));
    trigger_mask=0;read_count=0;press_during_rearm=true;
    assert(!Dev_LimitSwitch_TryRearmInterrupt(DEV_JOINT_M1));
    assert(test_exti.IMR==4 && !(test_exti.FTSR&M1_STOP_Pin));
    puts("limit_irq_port_tests: passed");return 0;
}
