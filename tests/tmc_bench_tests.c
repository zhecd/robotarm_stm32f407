#include "../tools/tmc_bench/bench_logic.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* 独立台架测试失败时直接退出，避免Windows CRT断言弹窗阻塞自动验证。 */
#undef assert
#define assert(expr) do { if(!(expr)) { fprintf(stderr,"check failed line %d: %s\n",__LINE__,#expr); exit(1); } } while(0)
int main(void)
{
    BenchGuard g={0};
    assert(!Bench_Expired(&g, 5000));
    assert(Bench_Start(&g, UINT32_MAX-400));
    assert(!Bench_Expired(&g, 598));
    assert(Bench_Expired(&g, 599));
    Bench_Stop(&g); assert(!g.active && g.used);
    assert(!Bench_Start(&g, 1234)); /* 停止/掉包均不能自动重试。 */
    BenchGuard step={0};
    assert(Bench_Start(&step,UINT32_MAX-400));
    unsigned rising=0; bool previous=false;
    for(uint32_t delta=0;delta<=BENCH_RUN_MS;delta++) {
        bool high=Bench_StepLevel(&step,step.started+delta);
        if(high && !previous)++rising;
        previous=high;
    }
    assert(rising==500 && !previous);
    assert(!Bench_StepLevel(&step,step.started+BENCH_RUN_MS+1));
    Bench_Stop(&step); assert(!Bench_StepLevel(&step,step.started+1));
    uint8_t f[8];
    const uint8_t read0[]={5,0,0};
    assert(Bench_Crc(read0,3)==0x48);
    const uint8_t stop[]={5,0,0xA2,0,0,0,0,0x0E};
    Bench_WriteFrame(0,0x22,0,f);
    assert(memcmp(f,stop,8)==0);
    for (unsigned n=0;n<4;n++) {
        Bench_WriteFrame((uint8_t)n,0x22,BENCH_VELOCITY,f);
        assert(f[0]==5 && f[1]==n && f[2]==0xA2);
        assert(f[3]==0 && f[4]==0 && f[5]==0x17 && f[6]==0x70);
    }
    puts("bench: wraparound deadline, one-shot latch, independent CRC vectors passed");
}
