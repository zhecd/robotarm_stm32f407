/** 坐标系有效性回归：协议模式READY不等于当前零位仍有效。
 * 使用真实协议/状态/安全服务，仅替换设备；失败返回码便于旧代码负对照。 */
#include "app/app_serial_protocol.h"
#include "state_service.h"
#include "safety_service.h"
#include "motion_service.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include <stdio.h>
#include <string.h>
static void Query(const char *line)
{FakeHardware_ClearTx();FakeHardware_SendLine(line);AppSerial_Service();}
#define CHECK(c) do {if(!(c)){fprintf(stderr,"reference check failed: %s\n",#c);return 2;}}while(0)
int main(int argc,char **argv)
{
    if(argc!=2)return 3;
    const char *scenario=argv[1];bool raw=!strcmp(scenario,"raw");
    bool valid=!strcmp(scenario,"valid") || !strcmp(scenario,"restored");
    FakeRobot_Reset();FakeHardware_Reset();StateService_Init();SafetyService_Init();MotionService_Init();
    AppSerial_Init(raw?APP_SERIAL_DIAGNOSTIC:APP_SERIAL_READY);
    if(strcmp(scenario,"unhomed")){
        SafetyService_MarkHomed();SafetyService_ClearAfterSuccessfulHoming();
        if(strcmp(scenario,"valid"))SafetyService_RevokePermission();
    }
    if(!strcmp(scenario,"fault")){
        SafetyService_MarkHomed();SafetyService_ClearAfterSuccessfulHoming();
        SafetyService_ReportHostStop();
    }
    if(!strcmp(scenario,"restored")){
        /* 只测试安全状态事务；真实M999重新回零另有运行集成测试。 */
        SafetyService_MarkHomed();SafetyService_ClearAfterSuccessfulHoming();
    }
    if(!strcmp(scenario,"not_ready"))AppSerial_SetMode(APP_SERIAL_FAILED);
    StateService_PublishAxisSampleTimed(0,12.0f,34.0f,0,0);
    Query("M409");
    CHECK(strstr(fake_tx,raw?"FRAME:RAW":valid?"FRAME:JOINT":"FRAME:UNAVAILABLE"));
    CHECK(strstr(fake_tx,"MOTOR:12,0,0")); /* 失去参考仍可读取原始反馈。 */
    Query("M114");
    if(raw)CHECK(strstr(fake_tx,"M114 RAWENC:"));
    else if(valid)CHECK(strstr(fake_tx,"M114 PLAN "));
    else CHECK(strstr(fake_tx,"error:") && !strstr(fake_tx,"M114 PLAN "));
    CHECK(!fake_joints_enabled && !fake_steps[0] && !fake_steps[1] && !fake_steps[2]);
    CHECK(!fake_gripper_open_count && !fake_gripper_close_count);
    printf("reference protocol %s passed\n",scenario);return 0;
}
