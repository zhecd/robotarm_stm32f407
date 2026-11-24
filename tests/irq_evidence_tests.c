/** 替身模拟pending/W1C和硬件调用，其余记录包装器为真实源码。 */
#include "main.h"
#include "irq_evidence.h"
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
TestExti test_exti;TestGpio test_gpio;TestDwt test_dwt;TestSyscfg test_syscfg;TestScb test_scb;
static unsigned callback_calls[3],vector_calls,hal_calls;
static uint32_t tick;
static bool stuck;
uint32_t HAL_GetTick(void){return tick;}
void __wrap_App_ISR_OnLimitVectorEnter(void);
void __wrap_HAL_GPIO_EXTI_IRQHandler(uint16_t pin);
void __wrap_App_ISR_OnGpioEdge(uint16_t pin);
void __wrap_App_ISR_OnLimitVectorExit(void);
void __real_App_ISR_OnLimitVectorEnter(void){++vector_calls;}
void __real_App_ISR_OnLimitVectorExit(void){}
void __real_App_ISR_OnGpioEdge(uint16_t pin)
{unsigned a=pin==M1_STOP_Pin?0:pin==M2_STOP_Pin?1:2;++callback_calls[a];}
void __real_HAL_GPIO_EXTI_IRQHandler(uint16_t pin)
{
    ++hal_calls;
    if(test_exti.PR&pin){
        if(!stuck){test_exti.PR&=~(uint32_t)pin;test_exti.SWIER&=~(uint32_t)pin;}
        __wrap_App_ISR_OnGpioEdge(pin);
    }
    ++test_dwt.CYCCNT;
}
static void Vector(uint32_t pending)
{
    test_exti.PR=pending;__wrap_App_ISR_OnLimitVectorEnter();
    /* 尚未写完不能发布半条记录。 */
    uint32_t n=g_irq_evidence.complete_count;
    __wrap_HAL_GPIO_EXTI_IRQHandler(M1_STOP_Pin);assert(g_irq_evidence.complete_count==n);
    __wrap_HAL_GPIO_EXTI_IRQHandler(M2_STOP_Pin);assert(g_irq_evidence.complete_count==n);
    __wrap_HAL_GPIO_EXTI_IRQHandler(M3_STOP_Pin);
    assert(g_irq_evidence.complete_count==n);
    __wrap_App_ISR_OnLimitVectorExit();
}
int main(void)
{
    test_exti.IMR=test_exti.FTSR=0x1c00;test_gpio.IDR=0x1c00;
    test_syscfg.EXTICR[2]=0x2200;test_syscfg.EXTICR[3]=2;test_scb.ICSR=56;
    Vector(0x1c00);
    assert(g_irq_evidence.complete_count==1 && vector_calls==1 && hal_calls==3);
    IrqEvidenceRecord r=g_irq_records[0];
    assert(r.pending_enter==0x1c00 && r.mask_enter==0x1c00 && r.gpio_c==0x1c00);
    assert(r.before[0]==0x1c00 && r.after[0]==0x1800 && r.after[1]==0x1000 && r.after[2]==0);
    assert(r.callback_mask==7 && r.exticr3==0x2200 && r.exticr4==2 && r.vector==56);
    assert(callback_calls[0]==1 && callback_calls[1]==1 && callback_calls[2]==1);
    stuck=true;tick=100;Vector(0x800);assert(g_irq_records[1].after[1]==0x800);
    assert(g_irq_records[1].callback_mask==2);stuck=false;
    /* 只有M1，及零分支的共享向量，也必须正常发布。 */
    test_exti.PR=0x400;__wrap_App_ISR_OnLimitVectorEnter();
    __wrap_HAL_GPIO_EXTI_IRQHandler(0x400);__wrap_App_ISR_OnLimitVectorExit();
    assert(g_irq_evidence.complete_count==3 && g_irq_records[2].callback_mask==1);
    __wrap_App_ISR_OnLimitVectorEnter();__wrap_App_ISR_OnLimitVectorExit();
    assert(g_irq_evidence.complete_count==4 && g_irq_records[3].callback_mask==0);
    for(unsigned i=4;i<IRQ_EVIDENCE_CAPACITY;i++)Vector(0);
    IrqEvidenceRecord first=g_irq_records[0];
    Vector(0x1000);assert(g_irq_evidence.complete_count==64 && g_irq_evidence.dropped==1);
    assert(g_irq_records[0].pending_enter==first.pending_enter && g_irq_records[0].after[2]==first.after[2]);
    assert(g_irq_evidence.callbacks[0]==2 && g_irq_evidence.callbacks[1]==2 && g_irq_evidence.callbacks[2]==2);
    assert(hal_calls==65*3-5 && vector_calls==65); /* 满后继续真实回调，不吞中断。 */
    puts("IRQ evidence: pre/post clear, axis mapping, publication, frozen history and real forwarding passed");return 0;
}
