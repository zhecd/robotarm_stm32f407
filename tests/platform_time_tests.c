/** 真实平台时钟回归：CPU周期回绕应保持微秒差值。 */
#include "main.h"
#include "platform_time.h"
#include "platform_critical.h"
#include "device/dev_joint.h"
#include "service/svc_homing.h"
#include "robot_config.h"
#include <stdio.h>
TestCoreDebug test_debug;
TestDwt test_dwt;
uint32_t SystemCoreClock=168000000U;
static uint32_t tick, depth, steps[3];
uint32_t HAL_GetTick(void) { return tick; }
PlatformCriticalState_t PlatformCritical_Enter(void) { return depth++; }
void PlatformCritical_Exit(PlatformCriticalState_t state)
{ if (depth!=state+1U) { fputs("FAIL: time critical section nesting\n",stderr); } depth=state; }
bool Dev_Joint_SetDirection(DevJointId_t axis,bool dir) { (void)dir;return axis<3; }
bool Dev_Joint_SetDirectionLevel(DevJointId_t axis,bool high) { (void)high;return axis<3; }
bool Dev_LimitSwitch_IsTriggered(DevJointId_t axis) { (void)axis;return false; }
bool Dev_Joint_Step(DevJointId_t axis) { if (axis>=3) return false; ++steps[axis]; return true; }
#define CHECK(expr) do { if (!(expr)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#expr);return 1; } } while (0)
int main(void)
{
    PlatformTime_OnTickFromISR(); CHECK(PlatformTime_NowUs()==0U);
    PlatformTime_Init();
    CHECK((CoreDebug->DEMCR&CoreDebug_DEMCR_TRCENA_Msk)!=0U);
    CHECK((DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)!=0U);
    DWT->CYCCNT=UINT32_MAX-83U;
    uint32_t before=PlatformTime_NowUs();
    DWT->CYCCNT=84U;
    uint32_t elapsed=PlatformTime_NowUs()-before;
    if (elapsed!=1U) {
        fprintf(stderr,"FAIL: 168 CPU cycles across CYCCNT wrap became %lu us, expected 1 us\n",
                (unsigned long)elapsed);
        return 1;
    }
    /* 1周期多次读取不得丢掉余数。 */
    PlatformTime_Init();
    for (unsigned i=1;i<=1680;i++) { DWT->CYCCNT=i; CHECK(PlatformTime_NowUs()==i/168U); }
    /* 业务长期不读NowUs，只由SysTick维持；跨多次DWT及32位微秒回绕。 */
    PlatformTime_Init();
    uint64_t cycles=0;
    for (unsigned i=0;i<220;i++) {
        cycles+=3360000000ULL; /* 每段20秒，仍小于一次CYCCNT回绕。 */
        DWT->CYCCNT=(uint32_t)cycles; PlatformTime_OnTickFromISR();
    }
    CHECK(PlatformTime_NowUs()==(uint32_t)(cycles/168U));
    tick=UINT32_MAX; CHECK(PlatformTime_NowMs()==UINT32_MAX);
    tick=0; CHECK(PlatformTime_NowMs()==0);
    /* 真实Homing与真实时间适配组合，不用理想化的微秒替身掩盖周期回绕。 */
    PlatformTime_Init();
    DWT->CYCCNT=UINT32_MAX-(HOMING_STEP_DELAY_US/2U)*168U;
    PlatformTime_OnTickFromISR(); CHECK(Svc_Homing_Start());
    DWT->CYCCNT+=(HOMING_STEP_DELAY_US/2U)*168U;
    Svc_Homing_Step(); CHECK(steps[0]==0 && steps[1]==0 && steps[2]==0);
    DWT->CYCCNT+=(HOMING_STEP_DELAY_US-HOMING_STEP_DELAY_US/2U)*168U;
    Svc_Homing_Step(); CHECK(steps[0]==1 && steps[1]==1 && steps[2]==1);
    Svc_Homing_Cancel();
    SystemCoreClock=84000000U; PlatformTime_Init(); DWT->CYCCNT=84U;
    CHECK(PlatformTime_NowUs()==1U);
    SystemCoreClock=0; PlatformTime_Init(); DWT->CYCCNT=1000U;
    CHECK(PlatformTime_NowUs()==0U && depth==0U);
    puts("platform_time_tests: passed");return 0;
}
