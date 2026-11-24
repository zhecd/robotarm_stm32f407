/** @file m1_reference_tests.c
 * @brief 用真实M1状态机驱动参考生命周期；无串口、HAL或实机输出。
 */
#include "m1_reference.h"
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1); } } while(0)

static M1Test motion;
static M1Reference ref;
static M1Inputs inputs;
static uint32_t now;

static void Reset(uint32_t start)
{
    now=start;M1Test_Init(&motion);M1Reference_Init(&ref,4.5f);
    inputs=(M1Inputs){.feedback_valid=true,.main_seen_ms=now};
}
static void Begin(bool home,bool high)
{
    if (home) CHECK(M1Test_ConfirmDirection(&motion,high));
    CHECK(M1Test_Start(&motion,now,home,high,true,true,&inputs));
}
static void Tick(void)
{
    ++now;inputs.main_seen_ms=now;
    if (M1Test_Active(&motion) && (uint32_t)(now-motion.lease_ms)>=100U)
        CHECK(M1Test_KeepAlive(&motion,now));
    M1Test_Tick(&motion,now,&inputs);
    M1Reference_ObserveState(&ref,&motion,now);
}
static void Sample(uint32_t started,uint32_t finished,float angle,bool valid,uint8_t mask,bool config)
{
    M1ReferenceSample s={.started_ms=started,.finished_ms=finished,.motor_deg=angle,
        .valid=valid,.limit_mask=mask,.input_config_ok=config};
    M1Reference_OnSample(&ref,&motion,&s);
}
static void FinishHome(uint32_t start,bool high,unsigned seek)
{
    Reset(start);Begin(true,high);
    while (motion.seek_steps<seek) Tick();
    inputs.m1_limit=true;
    while (motion.state!=M1_BACKOFF) Tick();
    CHECK(ref.state==M1_REF_UNSET); /* 触发点不能成为退回后原点。 */
    while (motion.backoff_steps<4U) Tick();
    inputs.m1_limit=false;
    while (M1Test_Active(&motion)) Tick();
    CHECK(motion.state==M1_DONE && ref.state==M1_REF_WAIT_SAMPLE);
}
static void Establish(void)
{
    FinishHome(0,false,12);Sample(now,now+2U,720.0f,true,0,true);now+=2U;
    CHECK(ref.state==M1_REF_VALID && ref.origin_motor_deg==720.0f);
}

static void Success(void)
{
    for (unsigned high=0;high<2;high++) {
        FinishHome(0,high,high?199:12);
        uint32_t done=now;
        /* 请求在DONE前开始、在DONE后完成，即使数值有效也不允许提交为原点。 */
        Sample(done-1U,done+1U,111.0f,true,0,true);
        CHECK(ref.state==M1_REF_WAIT_SAMPLE);
        Sample(done+2U,done+4U,720.0f,true,0,true);
        CHECK(ref.state==M1_REF_VALID && ref.reason==M1_REF_OK);
        CHECK(ref.origin_motor_deg==720.0f && ref.relative_joint_deg==0.0f);
        Sample(done+20U,done+22U,724.5f,true,0,true);
        CHECK(ref.relative_motor_deg==4.5f && ref.relative_joint_deg==1.0f);
        Sample(done+40U,done+42U,715.5f,true,0,true);
        CHECK(ref.relative_joint_deg==-1.0f && ref.origin_motor_deg==720.0f);
        /* 跨越360度仍按展开角计算，不能用fmod误丢一整圈。 */
        Sample(done+60U,done+62U,1080.0f,true,0,true);
        CHECK(ref.relative_joint_deg==80.0f);
        CHECK(!motion.enable && !motion.step && motion.used);
        CHECK(!M1Test_Start(&motion,done+62U,true,false,true,true,&inputs));
    }
}

static void Gating(void)
{
    Reset(0);Sample(0,1,123.0f,true,0,true);CHECK(ref.state==M1_REF_UNSET);
    Begin(false,false);while(M1Test_Active(&motion))Tick();
    CHECK(motion.state==M1_DONE && ref.state==M1_REF_UNSET); /* JOG成功不是HOME。 */
    FinishHome(0,false,12);M1Test good=motion;
    /* 单独破坏每条完成证据，均不能建立等待/有效参考。 */
    for(unsigned bad=0;bad<12;bad++) {
        motion=good;M1Reference_Init(&ref,4.5f);
        switch(bad) {
        case 0:motion.state=M1_SEEK;break;
        case 1:motion.reason=M1_LIMIT_STUCK;break;
        case 2:motion.used=false;break;
        case 3:motion.enable=true;break;
        case 4:motion.step=true;break;
        case 5:motion.direction_confirmed=false;break;
        case 6:motion.release_confirmed=false;break;
        case 7:motion.jog_steps=1;break;
        case 8:motion.seek_steps=0;break;
        case 9:motion.seek_steps=201;break;
        case 10:motion.backoff_steps=39;break;
        default:motion.backoff_steps=41;break;
        }
        M1Reference_ObserveState(&ref,&motion,now);
        Sample(now,now+1U,123.0f,true,0,true);CHECK(ref.state==M1_REF_UNSET);
    }
}

static void Timing(void)
{
    FinishHome(0,false,12);uint32_t done=now;
    M1Reference_ObserveState(&ref,&motion,done+99U);CHECK(ref.state==M1_REF_WAIT_SAMPLE);
    M1Reference_ObserveState(&ref,&motion,done+100U);
    CHECK(ref.state==M1_REF_INVALID && ref.reason==M1_REF_SAMPLE_TIMEOUT);
    Sample(done+101U,done+102U,1.0f,true,0,true);CHECK(ref.state==M1_REF_INVALID);
    FinishHome(0,false,12);done=now;
    Sample(done,done+99U,1.0f,true,0,true);CHECK(ref.state==M1_REF_VALID);
    M1Reference_ObserveState(&ref,&motion,done+100U);
    CHECK(ref.state==M1_REF_INVALID && ref.reason==M1_REF_BAD_FEEDBACK);
    FinishHome(0,false,12);done=now;
    Sample(done,done+100U,1.0f,true,0,true);CHECK(ref.state==M1_REF_INVALID);
    Establish();uint32_t sampled=ref.last_sample_started_ms;
    M1Reference_ObserveState(&ref,&motion,sampled+99U);CHECK(ref.state==M1_REF_VALID);
    M1Reference_ObserveState(&ref,&motion,sampled+100U);CHECK(ref.state==M1_REF_INVALID);
}

static void Faults(void)
{
    for(unsigned reason=M1_HOST_STOP;reason<=M1_INPUT_FAULT;reason++) {
        Establish();M1Test_Stop(&motion,(M1Reason)reason);
        M1Reference_ObserveState(&ref,&motion,now);
        CHECK(ref.state==M1_REF_INVALID && ref.reason==M1_REF_MOTION_FAULT);
        Sample(now,now+1U,10.0f,true,0,true);CHECK(ref.state==M1_REF_INVALID);
    }
    for(unsigned mask=1;mask<8;mask++) {
        Establish();Sample(now+20U,now+21U,720.0f,true,(uint8_t)mask,true);
        CHECK(ref.state==M1_REF_INVALID && ref.reason==M1_REF_LIMIT_ACTIVE);
    }
    Establish();Sample(now+20U,now+21U,720.0f,true,0,false);
    CHECK(ref.reason==M1_REF_BAD_CONFIG && ref.state==M1_REF_INVALID);
    Establish();M1Reference_ObserveState(&ref,NULL,now);CHECK(ref.state==M1_REF_INVALID);
    Establish();M1Reference_OnSample(&ref,&motion,NULL);CHECK(ref.state==M1_REF_INVALID);
    M1Reference_Init(NULL,4.5f);M1Reference_ObserveState(NULL,NULL,0);
    M1Reference_OnSample(NULL,NULL,NULL); /* 空上下文必须安全返回。 */
}

static void Numeric(void)
{
    const float invalid[]={0.0f,-4.5f,NAN,INFINITY,-INFINITY};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++) {
        M1Reference_Init(&ref,invalid[i]);CHECK(ref.state==M1_REF_INVALID && ref.reason==M1_REF_BAD_CONFIG);
    }
    const float angles[]={NAN,INFINITY,-INFINITY};
    for(unsigned i=0;i<sizeof(angles)/sizeof(angles[0]);i++) {
        Establish();Sample(now+20U,now+21U,angles[i],true,0,true);
        CHECK(ref.state==M1_REF_INVALID && ref.reason==M1_REF_BAD_FEEDBACK);
    }
    Establish();Sample(now+20U,now+21U,720.0f,false,0,true);CHECK(ref.state==M1_REF_INVALID);
    FinishHome(0,false,12);Sample(now,now+1U,FLT_MAX,true,0,true);
    Sample(now+20U,now+21U,-FLT_MAX,true,0,true);CHECK(ref.state==M1_REF_INVALID);
    FinishHome(0,false,12);M1Reference_Init(&ref,FLT_MIN);
    M1Reference_ObserveState(&ref,&motion,now);Sample(now,now+1U,0.0f,true,0,true);
    Sample(now+20U,now+21U,100.0f,true,0,true);CHECK(ref.state==M1_REF_INVALID);
}

static void Wrap(void)
{
    FinishHome(UINT32_MAX-100U,false,12); /* 状态机运行跨过HAL计数回绕。 */
    CHECK(ref.state==M1_REF_WAIT_SAMPLE);
    M1Reference_Init(&ref,4.5f);now=UINT32_MAX-5U;
    M1Reference_ObserveState(&ref,&motion,now);
    Sample(now-1U,now+1U,1.0f,true,0,true);CHECK(ref.state==M1_REF_WAIT_SAMPLE);
    Sample(now+2U,now+4U,720.0f,true,0,true);CHECK(ref.state==M1_REF_VALID);
    Sample(now+20U,now+22U,724.5f,true,0,true);
    CHECK(ref.state==M1_REF_VALID && ref.relative_joint_deg==1.0f);
    M1Reference_ObserveState(&ref,&motion,now+120U);CHECK(ref.state==M1_REF_INVALID);
}

static void Replay(void)
{
    Establish();uint32_t last=ref.last_sample_started_ms;
    Sample(last,last+20U,999.0f,true,0,true);CHECK(ref.relative_motor_deg==0.0f);
    Sample(last-1U,last+21U,999.0f,true,0,true);CHECK(ref.relative_motor_deg==0.0f);
    CHECK(ref.last_sample_started_ms==last);
    M1Reference_ObserveState(&ref,&motion,last+100U);CHECK(ref.state==M1_REF_INVALID);
    Sample(last+101U,last+102U,123.0f,true,0,true);CHECK(ref.state==M1_REF_INVALID);
    M1Reference_Init(&ref,4.5f);M1Test_Init(&motion);
    Sample(1000,1001,123.0f,true,0,true);CHECK(ref.state==M1_REF_UNSET); /* 模拟复位。 */
}

static void HomeFailures(void)
{
    /* 真实运动状态机的每种异常都不能意外提交参考，也不能自动重试。 */
    for(unsigned mode=0;mode<8;mode++) {
        Reset(0);Begin(true,false);
        switch(mode) {
        case 0:while(M1Test_Active(&motion))Tick();CHECK(motion.reason==M1_TRAVEL_BOUND);break;
        case 1:inputs.m1_limit=true;while(M1Test_Active(&motion))Tick();CHECK(motion.reason==M1_LIMIT_STUCK);break;
        case 2:inputs.m1_limit=true;Tick();inputs.m1_limit=false;Tick();CHECK(motion.reason==M1_LIMIT_UNSTABLE);break;
        case 3:inputs.other_limit=true;Tick();CHECK(motion.reason==M1_OTHER_LIMIT);break;
        case 4:inputs.feedback_valid=false;Tick();CHECK(motion.reason==M1_FEEDBACK_LOST);break;
        case 5:now=M1_LEASE_MS;inputs.main_seen_ms=now;M1Test_Tick(&motion,now,&inputs);CHECK(motion.reason==M1_HOST_LOST);break;
        case 6:now=M1_HOME_DEADLINE_MS;M1Test_Tick(&motion,now,&inputs);CHECK(motion.reason==M1_TIME_BOUND);break;
        default:M1Test_Stop(&motion,M1_HOST_STOP);break;
        }
        M1Reference_ObserveState(&ref,&motion,now);
        CHECK(ref.state==M1_REF_INVALID && !motion.enable && !motion.step);
        Sample(now,now+1U,720.0f,true,0,true);CHECK(ref.state==M1_REF_INVALID);
        CHECK(!M1Test_Start(&motion,now,true,false,true,true,&inputs));
    }
}

int main(int argc,char **argv)
{
    CHECK(argc==2);
    if(!strcmp(argv[1],"success"))Success();
    else if(!strcmp(argv[1],"gating"))Gating();
    else if(!strcmp(argv[1],"timing"))Timing();
    else if(!strcmp(argv[1],"faults"))Faults();
    else if(!strcmp(argv[1],"numeric"))Numeric();
    else if(!strcmp(argv[1],"wrap"))Wrap();
    else if(!strcmp(argv[1],"replay"))Replay();
    else if(!strcmp(argv[1],"home_failures"))HomeFailures();
    else CHECK(false);
    printf("m1_reference_%s: passed; no hardware access\n",argv[1]);return 0;
}
