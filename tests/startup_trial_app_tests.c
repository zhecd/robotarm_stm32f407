/** 实际正常App+回零Service，虚拟设备在已知步数触发开关；不模拟App成功。 */
#include "app_init.h"
#include "app/app_teleop.h"
#include "service/svc_homing.h"
#include "safety_service.h"
#include "fake_hardware.h"
#include "fake_robot.h"
#include "robot_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
bool AppTrial_ReleasePreparedStartup(void);
int main(int argc,char **argv)
{
    assert(argc==2);bool noseek=!strcmp(argv[1],"noseek"),stuck=!strcmp(argv[1],"stuck");
    FakeRobot_Reset();FakeHardware_Reset();App_Teleop_SetMode(SYS_MODE_GCODE);App_Init();
    FakeRobot_AdvanceUs(30000000U); /* 人工确认等待不消耗本次实际检查期限。 */
    assert(!fake_joints_enabled && !fake_gripper_init_count && !fake_steps[0]);
    if(!strcmp(argv[1],"fault")){
        SafetyService_ReportHostStop();assert(!AppTrial_ReleasePreparedStartup());return 0;
    }
    assert(AppTrial_ReleasePreparedStartup());App_RunOnce();
    assert(fake_gripper_init_count==1 && Svc_Homing_GetState()==SVC_HOMING_SEEK);
    assert(!AppTrial_ReleasePreparedStartup());
    const unsigned seek[3]={30,60,90};
    unsigned backs[3]={0},iterations=0;
    while(iterations++<4300 && !SafetyService_IsMotionAllowed() && !SafetyService_HasFault()){
        bool back=Svc_Homing_GetState()==SVC_HOMING_BACKOFF;
        for(unsigned a=0;a<3;a++){
            fake_limits[a]=back?stuck:(!noseek && fake_steps[a]>=seek[a]);
            backs[a]=fake_steps[a];
        }
        FakeRobot_AdvanceUs(HOMING_STEP_DELAY_US);App_RunOnce();
    }
    if(noseek){
        assert(Svc_Homing_GetState()==SVC_HOMING_FAILED && !fake_joints_enabled);
        for(unsigned a=0;a<3;a++)assert(fake_steps[a]==200);
    }else if(stuck){
        assert(Svc_Homing_GetState()==SVC_HOMING_FAILED && !fake_joints_enabled);
    }else{
        assert(SafetyService_IsMotionAllowed() && Svc_Homing_GetState()==SVC_HOMING_COMPLETE);
        assert(fake_steps[0]==seek[0]+4000 && fake_steps[1]==seek[1]+1240 && fake_steps[2]==seek[2]+200);
        assert(fake_timer_start_count==1); /* 真实试验guard在该端口抑制TIM6，正常App仍提交。 */
        assert(fake_gripper_service_count>=4000);
        FakeHardware_ClearTx();FakeHardware_SendLine("M408");App_RunOnce();
        assert(strstr(fake_tx,"STATE:READY HOMED:1 ALLOWED:1"));
    }
    (void)backs;
    puts("real startup low-speed profile passed");return 0;
}
