/** M1真实纯状态机：有限脉冲、限位确认、退回、停止、失联、回绕及拒绝重放。 */
#include "m1_logic.h"
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1); } } while (0)
static M1Test t;
static M1Inputs in;
static uint32_t now;
static void Reset(uint32_t start)
{ M1Test_Init(&t);now=start;in=(M1Inputs){.feedback_valid=true,.main_seen_ms=now}; }
static void Tick(bool alive)
{
    ++now;
    if (alive) { in.main_seen_ms=now;if ((uint32_t)(now-t.lease_ms)>=100) CHECK(M1Test_KeepAlive(&t,now)); }
    M1Test_Tick(&t,now,&in);
    CHECK(t.jog_steps<=M1_JOG_STEPS && t.seek_steps<=200 && t.backoff_steps<=40);
    CHECK(M1Test_Active(&t) || (!t.enable && !t.step));
}
static void Start(bool home)
{
    if (home) CHECK(M1Test_ConfirmDirection(&t,false));
    CHECK(M1Test_Start(&t,now,home,false,true,true,&in));
}
int main(void)
{
    /* 编译锁、未确认方向、配置/反馈/初始限位任一不合格都不得启动。 */
    for (unsigned mode=0;mode<6;mode++) {
        Reset(0);in.feedback_valid=mode!=1;in.m1_limit=mode==2;in.other_limit=mode==3;
        CHECK(!M1Test_Start(&t,now,mode==5,false,mode!=0,mode!=4,&in));
        CHECK(!t.enable && !t.step && !t.used);
    }
    for (unsigned high=0;high<2;high++) {
        Reset(UINT32_MAX-100);CHECK(M1Test_Start(&t,now,false,high,true,true,&in));
        bool previous=false;unsigned rises=0;uint32_t last=now;
        while (M1Test_Active(&t)) {
            Tick(true);
            if (t.step && !previous) { if (rises) CHECK((uint32_t)(now-last)==20);else CHECK((uint32_t)(now-(UINT32_MAX-100U))==2);last=now;++rises; }
            previous=t.step;
        }
        CHECK(rises==M1_JOG_STEPS && t.state==M1_DONE && t.dir_high==(bool)high);
        CHECK(!M1Test_Start(&t,now,false,false,true,true,&in));
    }
    Reset(0);Start(true);
    while (t.seek_steps<12) Tick(true);
    in.m1_limit=true;Tick(true);uint32_t seek=t.seek_steps;
    CHECK(t.state==M1_QUALIFY && !t.step);
    for (unsigned i=0;i<9;i++) { Tick(true);CHECK(t.seek_steps==seek && !t.step); }
    Tick(true);CHECK(t.state==M1_BACKOFF && t.dir_high && !t.step);
    while (t.backoff_steps<4) Tick(true);
    in.m1_limit=false;
    while (M1Test_Active(&t)) Tick(true);
    CHECK(t.state==M1_DONE && t.seek_steps==12 && t.backoff_steps==40 && t.release_confirmed);
    /* 寻限位两向和末端触发：确认阶段不加步，退回确实反向且只40步。 */
    for(unsigned high=0;high<2;high++) {
        Reset(UINT32_MAX-80U);CHECK(M1Test_ConfirmDirection(&t,high));
        CHECK(M1Test_Start(&t,now,true,false,true,true,&in));
        while(t.seek_steps<199U)Tick(true);
        in.m1_limit=true;Tick(true);uint32_t hold=t.seek_steps;
        while(t.state==M1_QUALIFY){CHECK(t.seek_steps==hold && !t.step);Tick(true);}
        CHECK(t.state==M1_BACKOFF && t.dir_high!=(bool)high && !t.step);
        while(t.backoff_steps<4U)Tick(true);
        in.m1_limit=false;while(M1Test_Active(&t))Tick(true);
        CHECK(t.state==M1_DONE && t.seek_steps==199U && t.backoff_steps==40U);
    }
    /* 覆盖整个HOME时间窗的主机中止：寻限位/确认/退回之后均不能再出脉冲。 */
    for(unsigned cut=0;cut<M1_HOME_DEADLINE_MS;cut++) {
        Reset(0);Start(true);
        while(now<cut && M1Test_Active(&t)) {
            if(t.state==M1_SEEK && t.seek_steps>=12U)in.m1_limit=true;
            if(t.state==M1_BACKOFF && t.backoff_steps>=4U)in.m1_limit=false;
            Tick(true);
        }
        M1Test_Stop(&t,M1_HOST_STOP);
        uint32_t seek=t.seek_steps,back=t.backoff_steps;
        for(unsigned i=0;i<100;i++)M1Test_Tick(&t,++now,&in);
        CHECK(!t.enable && !t.step && t.seek_steps==seek && t.backoff_steps==back);
        CHECK(!M1Test_KeepAlive(&t,now) && !M1Test_ConfirmDirection(&t,true));
    }
    Reset(0);Start(true);while(M1Test_Active(&t))Tick(true);
    CHECK(t.state==M1_FAILED && t.reason==M1_TRAVEL_BOUND && t.seek_steps==200);
    Reset(0);Start(true);in.m1_limit=true;
    while(M1Test_Active(&t))Tick(true);
    CHECK(t.reason==M1_LIMIT_STUCK && t.backoff_steps==40);
    Reset(0);Start(true);in.m1_limit=true;Tick(true);in.m1_limit=false;Tick(true);
    CHECK(t.reason==M1_LIMIT_UNSTABLE && !t.step);
    Reset(0);Start(false);Tick(true);Tick(true);in.m1_limit=true;Tick(true);
    CHECK(t.reason==M1_LIMIT_DURING_JOG && !t.enable && !t.step);
    Reset(0);Start(false);in.other_limit=true;Tick(true);CHECK(t.reason==M1_OTHER_LIMIT);
    Reset(0);Start(false);in.feedback_valid=false;Tick(true);CHECK(t.reason==M1_FEEDBACK_LOST);
    Reset(0);Start(true);now=250;M1Test_Tick(&t,now,&in);CHECK(t.reason==M1_MAIN_STALLED);
    Reset(0);Start(true);now=500;in.main_seen_ms=now;
    CHECK(!M1Test_KeepAlive(&t,now));M1Test_Tick(&t,now,&in);CHECK(t.reason==M1_HOST_LOST);
    Reset(0);Start(true);now=5500;in.main_seen_ms=now;
    M1Test_Tick(&t,now,&in);CHECK(t.reason==M1_TIME_BOUND && !t.step);
    /* 不补发迟到脉冲；总界限和一次动作锁不能被稀疏调用绕过。 */
    Reset(0);Start(false);now=101;in.main_seen_ms=now;CHECK(M1Test_KeepAlive(&t,now));
    M1Test_Tick(&t,now,&in);CHECK(t.jog_steps==1);
    M1Test_Stop(&t,M1_HOST_STOP);for(unsigned i=0;i<1000;i++)M1Test_Tick(&t,++now,&in);
    CHECK(t.reason==M1_HOST_STOP && t.jog_steps==1 && !t.enable && !t.step);
    CHECK(!M1Test_KeepAlive(&t,now) && !M1Test_ConfirmDirection(&t,true));
    /* 对每个可能停止时刻逐个中止；之后100个tick都不能再发脉冲。 */
    for (unsigned cut=0;cut<M1_JOG_DEADLINE_MS;cut++) {
        Reset(0);Start(false);
        for(unsigned i=0;i<cut && M1Test_Active(&t);i++)Tick(true);
        M1Test_Stop(&t,M1_HOST_STOP);unsigned count=t.jog_steps;
        for(unsigned i=0;i<100;i++)M1Test_Tick(&t,++now,&in);
        CHECK(!t.enable && !t.step && t.jog_steps==count);
    }
    Reset(0);Start(false);now=M1_JOG_DEADLINE_MS;in.main_seen_ms=now;
    M1Test_Tick(&t,now,&in);CHECK(t.reason==M1_TIME_BOUND && !t.enable && !t.step);
    /* 专用8秒点动经过5.5秒仍活动，但HOME不能借此延长，以上HOME截止断言保留。 */
#if M1_JOG_JOINT_TEN_DEG
    Reset(0);Start(false);while(now<M1_HOME_DEADLINE_MS)Tick(true);
    CHECK(t.state==M1_JOG && t.enable && t.jog_steps<400);
    while(M1Test_Active(&t))Tick(true);
    CHECK(t.state==M1_DONE && t.jog_steps==400 && now<9000);
#endif
    printf("m1_homing_logic_tests: passed; steps=%u; %u stop offsets; no physical motion tested\n",M1_JOG_STEPS,M1_JOG_DEADLINE_MS);return 0;
}
