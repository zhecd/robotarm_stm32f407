/** 编译真实记录器；替身只负责寄存器、时基、HAL清除和回调。
 * 不模拟真实NVIC/APB或机械触点，不在测试里放宽过载预算。 */
#include "main.h"
#include "irq_evidence.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#ifndef TEST_AXIS_AVAILABLE
#define TEST_AXIS_AVAILABLE 1
#endif
#define CHECK(x) do{if(!(x)){fprintf(stderr,"axis evidence line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
TestExti test_exti;TestGpio test_gpio;TestDwt test_dwt;TestSyscfg test_syscfg;TestScb test_scb;
static uint32_t callbacks[3],vectors,hal_calls,tick,mask_at,mask_axis;
uint32_t HAL_GetTick(void){return tick;}
void __wrap_App_ISR_OnLimitVectorEnter(void);
void __wrap_App_ISR_OnLimitVectorExit(void);
void __wrap_HAL_GPIO_EXTI_IRQHandler(uint16_t pin);
void __wrap_App_ISR_OnGpioEdge(uint16_t pin);
void __real_App_ISR_OnLimitVectorEnter(void){++vectors;}
void __real_App_ISR_OnLimitVectorExit(void){}
void __real_App_ISR_OnGpioEdge(uint16_t pin)
{
    unsigned a=pin==M1_STOP_Pin?0U:pin==M2_STOP_Pin?1U:2U;++callbacks[a];
    if(mask_at && a==mask_axis && callbacks[a]==mask_at)test_exti.IMR&=~(uint32_t)pin;
}
void __real_HAL_GPIO_EXTI_IRQHandler(uint16_t pin)
{
    ++hal_calls;
    if(test_exti.PR&pin){test_exti.PR&=~(uint32_t)pin;__wrap_App_ISR_OnGpioEdge(pin);}
    ++test_dwt.CYCCNT;
}
static void Vector(uint32_t pending)
{
    test_exti.PR=pending;++tick;__wrap_App_ISR_OnLimitVectorEnter();
    for(unsigned a=0;a<3;a++)__wrap_HAL_GPIO_EXTI_IRQHandler((uint16_t)(1U<<(10U+a)));
    __wrap_App_ISR_OnLimitVectorExit();
}
#if TEST_AXIS_AVAILABLE
static IrqAxisRecord Last(unsigned a)
{
    volatile IrqAxisHeader *h=&g_irq_axis_evidence[a];
    unsigned i=h->tail_count?IRQ_AXIS_HEAD_CAPACITY+(h->tail_next+31U)%32U:h->head_count-1U;
    return g_irq_axis_records[a][i];
}
#endif
int main(int argc,char **argv)
{
    CHECK(argc==2 || argc==3);const char *mode=argv[1];
    test_exti.IMR=test_exti.FTSR=0x1c00;test_gpio.IDR=0x1c00;
    test_syscfg.EXTICR[2]=0x2200;test_syscfg.EXTICR[3]=2;test_scb.ICSR=56;
    if(!strcmp(mode,"fairness")){
        for(unsigned n=0;n<300;n++)Vector(M1_STOP_Pin);
        Vector(M2_STOP_Pin);Vector(M3_STOP_Pin);
#if TEST_AXIS_AVAILABLE
        CHECK(g_irq_axis_evidence[0].events_total==300 && g_irq_axis_evidence[1].events_total==1 && g_irq_axis_evidence[2].events_total==1);
        CHECK(g_irq_axis_records[1][0].record.callback_mask==2 && g_irq_axis_records[2][0].record.callback_mask==4);
        CHECK(g_irq_axis_records[1][0].vector_sequence==301 && g_irq_axis_records[2][0].vector_sequence==302);
#else
        unsigned m2=0,m3=0;
        for(unsigned n=0;n<g_irq_evidence.complete_count;n++){m2+=!!(g_irq_records[n].callback_mask&2);m3+=!!(g_irq_records[n].callback_mask&4);}
        printf("legacy retained M2=%u M3=%u after M1 flood; real callbacks=%u/%u/%u\n",m2,m3,callbacks[0],callbacks[1],callbacks[2]);
        CHECK(m2>0 && m3>0); /* 旧全局缓冲负对照应明确失败，而非改断言接受数据丢失。 */
#endif
    }
#if TEST_AXIS_AVAILABLE
    else if(!strcmp(mode,"ring")){
        for(unsigned n=0;n<120;n++)Vector(M2_STOP_Pin);
        volatile IrqAxisHeader *h=&g_irq_axis_evidence[1];
        CHECK(h->head_count==32 && h->tail_count==32 && h->tail_overwrites==56 && h->tail_next==24);
        for(unsigned n=0;n<32;n++)CHECK(g_irq_axis_records[1][n].axis_sequence==n+1);
        for(unsigned n=0;n<32;n++)CHECK(g_irq_axis_records[1][32+(h->tail_next+n)%32].axis_sequence==89+n);
        CHECK(Last(1).axis_sequence==120 && !(h->revision&1));
    }else if(!strcmp(mode,"freeze")){
        mask_axis=2;mask_at=256;
        for(unsigned n=0;n<256;n++)Vector(M3_STOP_Pin);
        volatile IrqAxisHeader *h=&g_irq_axis_evidence[2];
        CHECK(h->frozen && h->freeze_vector==256 && !(h->freeze_imr&M3_STOP_Pin));
        CHECK(Last(2).axis_sequence==256 && h->tail_overwrites==192);
        IrqAxisRecord saved[IRQ_AXIS_CAPACITY];
        for(unsigned n=0;n<IRQ_AXIS_CAPACITY;n++)saved[n]=g_irq_axis_records[2][n];
        Vector(M3_STOP_Pin); /* 故意模拟调用者在屏蔽后仍转发；记录器不能吞真实回调。 */
        CHECK(callbacks[2]==257 && h->events_total==257 && h->ignored_after_freeze==1);
        for(unsigned n=0;n<IRQ_AXIS_CAPACITY;n++){IrqAxisRecord v=g_irq_axis_records[2][n];CHECK(!memcmp(&v,&saved[n],sizeof(v)));}
    }else if(!strcmp(mode,"shared")){
        for(unsigned n=0;n<80;n++)Vector(0x1c00);
        for(unsigned a=0;a<3;a++){
            IrqAxisRecord v=Last(a);CHECK(v.vector_sequence==80 && v.axis_sequence==80 && v.record.callback_mask==7);
            CHECK(v.record.before[0]==0x1c00 && v.record.after[0]==0x1800 && v.record.after[2]==0);
        }
    }else if(!strcmp(mode,"publish")){
        Vector(0);CHECK(g_irq_evidence.complete_count==1);
        for(unsigned a=0;a<3;a++)CHECK(g_irq_axis_evidence[a].head_count==0);
        test_exti.PR=M1_STOP_Pin;__wrap_App_ISR_OnLimitVectorEnter();
        __wrap_HAL_GPIO_EXTI_IRQHandler(M1_STOP_Pin);
        CHECK(g_irq_evidence.complete_count==1 && g_irq_axis_evidence[0].head_count==0);
        __wrap_App_ISR_OnLimitVectorExit();CHECK(g_irq_axis_evidence[0].head_count==1);
        CHECK(g_irq_axis_evidence[0].revision==2 && g_irq_records[1].callback_mask==1);
    }else if(!strcmp(mode,"transparent")){
        for(unsigned n=0;n<100;n++)Vector(n%8U<<10);
        CHECK(hal_calls==300 && vectors==100 && test_exti.IMR==0x1c00 && test_exti.FTSR==0x1c00 && test_exti.RTSR==0 && test_exti.PR==0);
        for(unsigned a=0;a<3;a++)CHECK(callbacks[a]==g_irq_evidence.callbacks[a] && callbacks[a]==g_irq_axis_evidence[a].events_total);
    }else if(!strcmp(mode,"saturation")){
        g_irq_evidence.vectors_total=UINT32_MAX;g_irq_evidence.callbacks[0]=UINT32_MAX;
        g_irq_axis_evidence[0].events_total=UINT32_MAX;g_irq_axis_evidence[0].revision=UINT32_MAX-1U;
        Vector(M1_STOP_Pin);
        CHECK(g_irq_evidence.vectors_total==UINT32_MAX && g_irq_evidence.callbacks[0]==UINT32_MAX);
        CHECK(g_irq_axis_evidence[0].events_total==UINT32_MAX && g_irq_axis_evidence[0].revision==0);
    }
#endif
    else CHECK(false);
#if TEST_AXIS_AVAILABLE
    if(argc==3){
        FILE *f=fopen(argv[2],"wb");CHECK(f!=NULL);
        for(unsigned a=0;a<3;a++){IrqAxisHeader h=g_irq_axis_evidence[a];CHECK(fwrite(&h,sizeof(h),1,f)==1);}
        for(unsigned a=0;a<3;a++)for(unsigned n=0;n<IRQ_AXIS_CAPACITY;n++){
            IrqAxisRecord v=g_irq_axis_records[a][n];CHECK(fwrite(&v,sizeof(v),1,f)==1);
        }
        CHECK(fclose(f)==0);
    }
#endif
    printf("axis evidence %s passed; no register masking by recorder\n",mode);return 0;
}
