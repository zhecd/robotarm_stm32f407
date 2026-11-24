/** 真实协议组合测试：任何启动/故障/诊断模式下M409只读，不触发执行器。 */
#include "app/app_serial_protocol.h"
#include "state_service.h"
#include "safety_service.h"
#include "motion_service.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
int main(int argc,char **argv)
{
    assert(argc==2);unsigned mode=(unsigned)strtoul(argv[1],NULL,10);assert(mode<=APP_SERIAL_OUTPUT_LOCKED);
    FakeRobot_Reset();FakeHardware_Reset();StateService_Init();SafetyService_Init();MotionService_Init();
    AppSerial_Init((AppSerialMode_t)mode);
    /* READY有效坐标用例必须先建立参考；无参考READY由专用回归覆盖。 */
    if(mode==APP_SERIAL_READY){SafetyService_MarkHomed();SafetyService_ClearAfterSuccessfulHoming();}
    StateService_PublishAxisSampleTimed(0,10,20,0,1);
    FakeRobot_SetTimeUs(100000);StateService_PublishAxisSampleTimed(1,30,40,99,100);
    FakeHardware_SendLine("M409");AppSerial_Service();
    assert(strstr(fake_tx,"M409 GENERATION:2 NOW_MS:100"));
    assert(strstr(fake_tx,"PRESENT_MASK:3 VALID_MASK:3 FRESH_MASK:2 MAX_AGE_MS:100"));
    assert(strstr(fake_tx,"AGE_MS:100,1,4294967295 READ_MS:1,1,0"));
    assert(strstr(fake_tx,"MOTOR:10,30,0 JOINT:20,40,0"));
    const char *frame=mode==APP_SERIAL_READY?"FRAME:JOINT":
        (mode==APP_SERIAL_DIAGNOSTIC || mode==APP_SERIAL_WAVEFORM)?"FRAME:RAW":"FRAME:UNAVAILABLE";
    assert(strstr(fake_tx,frame));
    FakeHardware_ClearTx();FakeHardware_SendLine("M115");AppSerial_Service();assert(strstr(fake_tx,"FEEDBACK_AGE"));
    FakeHardware_ClearTx();FakeHardware_SendLine("M409 X1");AppSerial_Service();assert(strstr(fake_tx,"error:"));
    /* 最长时间戳和科学计数角度仍能形成一条完整回复。 */
    FakeRobot_SetTimeUs((uint64_t)UINT32_MAX*1000U);
    for(unsigned i=0;i<3;i++)StateService_PublishAxisSampleTimed(i,FLT_MAX,-FLT_MAX,0,INT32_MAX);
    FakeHardware_ClearTx();FakeHardware_SendLine("M409");AppSerial_Service();
    assert(strstr(fake_tx,"M409 ") && !strstr(fake_tx,"response too long") && strlen(fake_tx)<384);
    assert(!fake_joints_enabled && fake_enable_true_count==0);
    assert(fake_steps[0]==0 && fake_steps[1]==0 && fake_steps[2]==0);
    assert(fake_gripper_open_count==0 && fake_gripper_close_count==0);
    puts("feedback_protocol_tests: passed");return 0;
}
