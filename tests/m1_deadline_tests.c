/** 真实截止端口 + 两级pending夹具，检查残余IRQ/真实更新事件/非法期限。 */
#include "m1_deadline.h"
#include "m1_logic.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(c) do{if(!(c)){fprintf(stderr,"line %d: %s\n",__LINE__,#c);exit(1);}}while(0)
TestDeadlineTimer deadline_timer;
static unsigned pending,clears,barriers;
void DeadlineTest_Barrier(void){
    CHECK(deadline_timer.DIER==0 && deadline_timer.CR1==0);
    CHECK(deadline_timer.SR==0 && deadline_timer.EGR==1 && deadline_timer.CNT==0);
    ++barriers;
}
void HAL_NVIC_ClearPendingIRQ(int irq){
    CHECK(irq==50 && deadline_timer.DIER==0 && deadline_timer.CR1==0 && deadline_timer.SR==0);
    pending=0;++clears;
}
int main(void)
{
    /* 外围UIF已经清除，NVIC仍锁存：初版无条件ISR会误停，修复后必须拒绝。 */
    deadline_timer=(TestDeadlineTimer){.CR1=1,.DIER=1,.SR=0,.CNT=0};pending=1;
    CHECK(!M1Deadline_ExpiredFromISR());CHECK(pending==1 && deadline_timer.CR1==1);
    CHECK(M1Deadline_Prepare(500));CHECK(!pending && clears==1 && barriers==1);
    CHECK(deadline_timer.ARR==4999 && deadline_timer.PSC==8399 && deadline_timer.CNT==0);
    CHECK(deadline_timer.DIER==1 && deadline_timer.CR1==0 && deadline_timer.SR==0);
    CHECK(!M1Deadline_ExpiredFromISR());
    /* 真正更新不能被过滤；停止以后相同IRQ不能再次报告。 */
    deadline_timer.CR1=1;deadline_timer.SR=1;
    CHECK(M1Deadline_ExpiredFromISR());CHECK(!deadline_timer.CR1 && !deadline_timer.DIER && !deadline_timer.SR);
    CHECK(!M1Deadline_ExpiredFromISR());
    CHECK(M1Deadline_Prepare(5500));CHECK(deadline_timer.ARR==54999);
    CHECK(M1Deadline_Prepare(2000));CHECK(deadline_timer.ARR==19999);
    CHECK(M1Deadline_Prepare(M1_TIMER_MAX_MS));CHECK(deadline_timer.ARR==M1_TIMER_MAX_MS*10U-1U);
    CHECK(M1Deadline_Prepare(1));CHECK(deadline_timer.ARR==9);
    for(unsigned i=0;i<1000;i++){
        deadline_timer.CR1=1;deadline_timer.SR=1;pending=1;
        CHECK(M1Deadline_Prepare(500));CHECK(!pending && !deadline_timer.SR && !deadline_timer.CR1);
        CHECK(!M1Deadline_ExpiredFromISR());
    }
    const uint32_t invalid[]={0,M1_TIMER_MAX_MS+1U,0xffffffffU};
    for(unsigned i=0;i<3;i++){
        deadline_timer.CR1=1;deadline_timer.DIER=1;deadline_timer.SR=1;pending=1;
        CHECK(!M1Deadline_Prepare(invalid[i]));CHECK(!pending && !deadline_timer.CR1 && !deadline_timer.DIER);
    }
    deadline_timer.SR=1;deadline_timer.DIER=0;
    CHECK(!M1Deadline_ExpiredFromISR()); /* 未启用来源不能当作截止。 */
    puts("m1_deadline_tests passed");return 0;
}
