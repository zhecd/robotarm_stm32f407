/** 包装器保持真实HAL分发/清除和真实App回调；仅追加固定长度RAM记录。
 * ISR没有打印、延时、I2C或动态分配。GPIO/pending依次读取，不能当作示波器。
 */
#include "main.h"
#include "irq_evidence.h"
#include <stdbool.h>
_Static_assert(sizeof(IrqEvidenceRecord)==76U,"IRQ evidence ABI: 19 words");
_Static_assert(sizeof(IrqEvidenceHeader)==36U,"IRQ header ABI: 9 words");
volatile IrqEvidenceHeader g_irq_evidence={0x49525132U,IRQ_EVIDENCE_CAPACITY,76U,0,0,0,{0,0,0}};
volatile IrqEvidenceRecord g_irq_records[IRQ_EVIDENCE_CAPACITY];
_Static_assert(sizeof(IrqAxisRecord)==84U,"axis record ABI: 21 words");
_Static_assert(sizeof(IrqAxisHeader)==60U,"axis header ABI: 15 words");
#define AXIS_HEADER(a) {.magic=IRQ_AXIS_MAGIC,.axis=(a),.head_capacity=IRQ_AXIS_HEAD_CAPACITY, \
    .tail_capacity=IRQ_AXIS_TAIL_CAPACITY,.record_bytes=sizeof(IrqAxisRecord)}
volatile IrqAxisHeader g_irq_axis_evidence[3]={AXIS_HEADER(0),AXIS_HEADER(1),AXIS_HEADER(2)};
volatile IrqAxisRecord g_irq_axis_records[3][IRQ_AXIS_CAPACITY];
/* 该共享向量为同优先级单写者。即使全局64槽满，也完成当前向量供各轴选择。
 * 存储和循环均有固定上限；ISR不分配内存、不打印、不改变EXTI寄存器。 */
static IrqEvidenceRecord current;
static bool vector_active;
static void Add(volatile uint32_t *value){if(*value!=UINT32_MAX)++*value;}
static int Axis(uint16_t pin)
{return pin==M1_STOP_Pin?0:pin==M2_STOP_Pin?1:pin==M3_STOP_Pin?2:-1;}
void __real_App_ISR_OnLimitVectorEnter(void);
void __wrap_App_ISR_OnLimitVectorEnter(void)
{
    Add(&g_irq_evidence.vectors_total);
    current=(IrqEvidenceRecord){0};current.tick_ms=HAL_GetTick();current.cycles_enter=DWT->CYCCNT;
    current.pending_enter=EXTI->PR;current.mask_enter=EXTI->IMR;current.gpio_c=GPIOC->IDR;
    current.exticr3=SYSCFG->EXTICR[2];current.exticr4=SYSCFG->EXTICR[3];
    current.rising=EXTI->RTSR;current.falling=EXTI->FTSR;current.software_trigger=EXTI->SWIER;
    current.vector=SCB->ICSR&0x1ffU;vector_active=true;
    __real_App_ISR_OnLimitVectorEnter();
}
void __real_HAL_GPIO_EXTI_IRQHandler(uint16_t pin);
void __wrap_HAL_GPIO_EXTI_IRQHandler(uint16_t pin)
{
    int a=Axis(pin);
    if(vector_active && a>=0)current.before[a]=EXTI->PR;
    __real_HAL_GPIO_EXTI_IRQHandler(pin);
    if(vector_active && a>=0){
        current.after[a]=EXTI->PR;
    }
}
static void PublishAxis(uint32_t axis)
{
    volatile IrqAxisHeader *h=&g_irq_axis_evidence[axis];
    ++h->revision;__DMB(); /* 奇数期间不接受主机读取为稳定快照。 */
    Add(&h->events_total);
    if(h->frozen){Add(&h->ignored_after_freeze);}
    else {
        uint32_t slot;
        bool head=h->head_count<IRQ_AXIS_HEAD_CAPACITY;
        if(head)slot=h->head_count;
        else slot=IRQ_AXIS_HEAD_CAPACITY+h->tail_next;
        IrqAxisRecord r={g_irq_evidence.vectors_total,h->events_total,current};
        g_irq_axis_records[axis][slot]=r;__DMB();
        if(head)++h->head_count;
        else {
            if(h->tail_count<IRQ_AXIS_TAIL_CAPACITY)++h->tail_count;
            else Add(&h->tail_overwrites);
            h->tail_next=(h->tail_next+1U)%IRQ_AXIS_TAIL_CAPACITY;
        }
        if((EXTI->IMR&(1U<<(10U+axis)))==0U){
            h->freeze_imr=EXTI->IMR;h->freeze_vector=g_irq_evidence.vectors_total;h->frozen=1U;
        }
    }
    __DMB();++h->revision;
}
void __real_App_ISR_OnLimitVectorExit(void);
void __wrap_App_ISR_OnLimitVectorExit(void)
{
    if(vector_active){
        current.cycles_exit=DWT->CYCCNT;
        for(uint32_t axis=0;axis<3U;++axis)
            if(current.callback_mask&(1U<<axis))PublishAxis(axis);
        uint32_t i=g_irq_evidence.complete_count;
        if(i<IRQ_EVIDENCE_CAPACITY){
            g_irq_records[i]=current;__DMB();g_irq_evidence.complete_count=i+1U;
        }else Add(&g_irq_evidence.dropped);
        vector_active=false;
    }
    __real_App_ISR_OnLimitVectorExit();
}
void __real_App_ISR_OnGpioEdge(uint16_t pin);
void __wrap_App_ISR_OnGpioEdge(uint16_t pin)
{
    int a=Axis(pin);
    if(a>=0){
        Add(&g_irq_evidence.callbacks[a]);
        if(vector_active)current.callback_mask|=1U<<a;
    }
    __real_App_ISR_OnGpioEdge(pin);
}
