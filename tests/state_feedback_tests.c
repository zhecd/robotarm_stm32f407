/** 真实StateService：逐轴年龄、失败、有限值、快照及时钟回绕。 */
#include "state_service.h"
#include "fake_robot.h"
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static void At(uint32_t ms){FakeRobot_SetTimeUs((uint64_t)ms*1000U);}
int main(int argc,char **argv)
{
    assert(argc==2);FakeRobot_Reset();StateService_Init();StateServiceStatus_t s;
    if(!strcmp(argv[1],"axes")) {
        StateService_GetStatus(&s);assert(StateService_FreshMask(&s,0,100)==0);
        StateService_PublishAxisSampleTimed(0,10,20,0,10);
        At(99);StateService_PublishAxisSampleTimed(1,30,40,99,99);
        StateService_GetStatus(&s);assert(s.timestamp_ms==99);
        assert(StateService_FreshMask(&s,99,100)==3);
        assert(StateService_FreshMask(&s,100,100)==2); /* M2刷新不能掩盖M1已老。 */
        StateService_PublishAxisReadFailure(1,3);StateService_GetStatus(&s);
        assert(!s.encoder_valid[1] && s.sample_started_ms[1]==99 && s.motor_angle_deg[1]==30);
        assert(StateService_FreshMask(&s,100,100)==0);
    } else if(!strcmp(argv[1],"invalid")) {
        StateService_PublishAxisSampleTimed(0,10,20,10,11);
        float bad[]={NAN,INFINITY,-INFINITY};
        for(unsigned i=0;i<3;i++) {
            StateService_PublishAxisSampleTimed(0,bad[i],20,20,21);
            StateService_GetStatus(&s);assert(!s.encoder_valid[0] && s.motor_angle_deg[0]==10);
            StateService_PublishAxisSampleTimed(0,10,bad[i],20,21);
        }
        StateService_PublishAxisSampleTimed(0,1,2,30,29);StateService_GetStatus(&s);
        assert(!s.encoder_valid[0] && s.sample_started_ms[0]==10);
        for(unsigned i=0;i<300;i++)StateService_PublishAxisSampleTimed(0,NAN,0,30,31);
        StateService_GetStatus(&s);assert(s.encoder_fail_count[0]==UINT8_MAX);
        uint32_t gen=s.generation;StateService_PublishAxisSampleTimed(3,1,2,0,1);
        StateService_PublishAxisReadFailure(255,1);StateService_GetStatus(&s);assert(s.generation==gen);
    } else if(!strcmp(argv[1],"wrap")) {
        StateService_PublishAxisSampleTimed(0,1,2,UINT32_MAX-10U,9U);StateService_GetStatus(&s);
        assert(s.encoder_valid[0] && StateService_FreshMask(&s,9U,100)==1);
        assert(StateService_FreshMask(&s,89U,100)==0);
        assert(StateService_FreshMask(&s,9U,0)==0 && StateService_FreshMask(&s,9U,UINT32_MAX)==0);
        assert(StateService_FreshMask(NULL,0,100)==0);
    } else if(!strcmp(argv[1],"slow")) {
        StateService_PublishAxisSampleTimed(0,1,2,10,110);At(110);StateService_GetStatus(&s);
        assert(s.encoder_valid[0] && StateService_FreshMask(&s,110,100)==0);
        StateServiceStatus_t old=s;StateService_PublishAxisReadFailure(0,1);StateService_GetStatus(&s);
        assert(old.encoder_valid[0] && !s.encoder_valid[0]);
        StateService_Init();StateService_GetStatus(&s);assert(!s.sample_present[0]);
    } else assert(false);
    puts("state_feedback_tests: passed");return 0;
}
