/** 枚举真实输出计划：任何选轴都不能把STEP发给另一轴或给夹爪PWM。 */
#include "axis_binding.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
static uint32_t Apply(uint32_t old,uint32_t bsrr)
{assert(((bsrr&0xffffU)&(bsrr>>16U))==0U);return (old&~(bsrr>>16U))|(bsrr&0xffffU);}
int main(int argc,char **argv)
{
    assert(argc==2);unsigned id=(unsigned)strtoul(argv[1],NULL,10);
    const AxisBinding *a=AxisBinding_Get(id);assert(a && a->id==id && a->index==id-1U && a->uart_node==id-1U);
    assert(a->limit_bit==(1U<<(id-1U)) && a->dir_d==(1U<<(id+2U)));
    assert(a->step_a==(id==1?1U<<6:id==2?1U<<7:0U));assert(a->step_b==(id==3?1U:0U));
    for(unsigned mask=0;mask<8;mask++) {
        M1Inputs in=AxisBinding_Inputs(id,(uint8_t)mask,true,123);
        assert(in.m1_limit==((mask&a->limit_bit)!=0U));
        assert(in.other_limit==((mask&(7U^a->limit_bit))!=0U));
        assert(in.feedback_valid && in.main_seen_ms==123);
    }
    /* 对所有低16位初始电平和5个控制布尔量，检查BSRR结果及无关引脚保留。 */
    for(unsigned flags=0;flags<32;flags++) {
        M1Test t={.enable=(flags&1)!=0,.step=(flags&2)!=0,.dir_high=(flags&4)!=0};
        bool permit=(flags&8)!=0,deadline=(flags&16)!=0;
        AxisOutputs o=AxisBinding_Outputs(id,&t,permit,deadline);
        bool on=t.enable && permit && !deadline;
        for(uint32_t old=0;old<65536U;old++) {
            uint32_t pa=Apply(old,o.a_bsrr),pb=Apply(old,o.b_bsrr),pd=Apply(old,o.d_bsrr);
            assert((pa&0xc2U)==(on&&t.step?a->step_a:0U));
            assert((pb&1U)==(on&&t.step?a->step_b:0U));
            assert(((pd&4U)==0U)==on && ((pd&a->dir_d)!=0U)==t.dir_high);
            assert((pa&~0xc2U)==(old&~0xc2U));assert((pb&~1U)==(old&~1U));
            assert((pd&~(4U|a->dir_d))==(old&~(4U|a->dir_d)));
        }
    }
    for(unsigned bad=0;bad<5;bad+=4) {
        assert(!AxisBinding_Get(bad));M1Test t={.enable=true,.step=true};
        AxisOutputs o=AxisBinding_Outputs(bad,&t,true,false);
        assert(Apply(0xffff,o.a_bsrr)==(0xffffU&~0xc2U));
        assert((Apply(0xffff,o.b_bsrr)&1U)==0U && (Apply(0,o.d_bsrr)&4U));
        M1Inputs in=AxisBinding_Inputs(bad,0,true,123);assert(!in.feedback_valid && in.other_limit);
    }
    AxisOutputs none=AxisBinding_Outputs(id,NULL,true,false);assert(Apply(0,none.d_bsrr)&4U);
    printf("axis_binding_%u: passed; 2097152 output states; no hardware\n",id);return 0;
}
