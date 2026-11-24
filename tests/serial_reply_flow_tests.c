/** 真实协议/命令/规划链，替换串口字节端口与设备时间；不连接板卡。 */
#include "app/app_serial_protocol.h"
#include "command_service.h"
#include "motion_service.h"
#include "safety_service.h"
#include "state_service.h"
#include "app/app_teleop.h"
#include "robot_config.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void Send(const char *line)
{FakeHardware_ClearTx();FakeHardware_SendLine(line);AppSerial_Service();}
#if defined(ROBOTARM_JOINT_RETURN_TEST) && ROBOTARM_JOINT_RETURN_TEST
static void AdvanceJointTrial(unsigned ms)
{
    for (unsigned i = 0; i < ms; ++i) {
        for (unsigned tick = 0; tick < TICKS_PER_MS; ++tick) MotionService_OnStepTickFromISR();
        FakeRobot_AdvanceUs(1000);
        MotionService_UpdateClosedLoop();
        AppSerial_Service();
    }
}

static void CheckJointTrial(void)
{
    assert(CommandService_Init(HOMEPOSE_X_MM, HOMEPOSE_Y_MM, HOMEPOSE_Z_MM) == ERR_OK);
    MotionService_SyncClosedLoopTarget();
    Send("M413 DOWN");
    assert(strstr(fake_tx, "error:") && MotionService_IsIdle());
    AdvanceJointTrial(CL_SETTLE_MS + 1U);
    Send("M413 UP");
    assert(strstr(fake_tx, "error:") && MotionService_IsIdle());

    Send("M413 DOWN");
    assert(strstr(fake_tx, "ok\r\n") && AppSerial_JointTrialActive());
    assert(MotionService_GetQueueCount() == 50U);
    Send("G1 X1");
    assert(strstr(fake_tx, "joint test must return"));
    Send("M413 UP");
    assert(strstr(fake_tx, "error:"));
    Send("M400");
    assert(!strstr(fake_tx, "ok\r\n"));
    AdvanceJointTrial(3000U);
    assert(!strstr(fake_tx, "ok\r\n"));
    fake_motor_deg[1] = -341.0f * DEGREES_PER_STEP;
    fake_motor_deg[2] = 1265.0f * DEGREES_PER_STEP;
    AdvanceJointTrial(100U + CL_SETTLE_MS + 5U);
    assert(strstr(fake_tx, "ok\r\n"));
    assert(fake_steps[0] == 0U && fake_steps[1] == 341U && fake_steps[2] == 1265U);
    int32_t m1, m2, m3;
    MotionService_GetTheorySteps(&m1, &m2, &m3);
    assert(m1 == 0 && m2 == -341 && m3 == 1265);

    Send("M413 UP");
    assert(strstr(fake_tx, "ok\r\n"));
    Send("M400");
    AdvanceJointTrial(3000U);
    assert(!strstr(fake_tx, "ok\r\n"));
    fake_motor_deg[1] = fake_motor_deg[2] = 0;
    AdvanceJointTrial(100U + CL_SETTLE_MS + 5U);
    assert(strstr(fake_tx, "ok\r\n") && !AppSerial_JointTrialActive());
    assert(fake_steps[0] == 0U && fake_steps[1] == 682U && fake_steps[2] == 2530U);
    MotionService_GetTheorySteps(&m1, &m2, &m3);
    assert(m1 == 0 && m2 == 0 && m3 == 0);
    Send("M413 DOWN");
    assert(strstr(fake_tx, "error:") && MotionService_IsIdle());
}
#endif
int main(int argc,char **argv)
{
    assert(argc==2);FakeRobot_Reset();FakeHardware_Reset();StateService_Init();SafetyService_Init();
    MotionService_Init();MotionService_InitClosedLoop();
    assert(CommandService_Init(0,185,240)==ERR_OK);
    SafetyService_MarkHomed();SafetyService_ClearAfterSuccessfulHoming();
    App_Teleop_SetMode(SYS_MODE_GCODE);AppSerial_Init(APP_SERIAL_READY);
#if defined(ROBOTARM_JOINT_RETURN_TEST) && ROBOTARM_JOINT_RETURN_TEST
    if (!strcmp(argv[1], "joint-trial")) {
        CheckJointTrial();
        puts("joint-trial passed");
        return 0;
    }
#endif
    if(!strcmp(argv[1],"queries")) {
        Send("M115");assert(strstr(fake_tx,"M115 FIRMWARE:RobotArm") && strstr(fake_tx,"UART:115200"));
        Send("M408");assert(strstr(fake_tx,"STATE:READY HOMED:1 ALLOWED:1 MODE:GCODE FAULT:0"));
        Send("M114");assert(strstr(fake_tx,"M114 PLAN X:0.00 Y:185.00 Z:240.00"));
        Send("M119");assert(strstr(fake_tx,"M1:OPEN M2:OPEN M3:OPEN"));
    } else if (!strcmp(argv[1], "settled")) {
        MotionService_SyncClosedLoopTarget();
        fake_motor_deg[2] = 8.0f;
        Send("M400");
        assert(!strstr(fake_tx, "ok\r\n"));
        FakeRobot_AdvanceUs(CL_SETTLE_MS * 1000U);
        MotionService_UpdateClosedLoop();
        assert(MotionService_IsClosedLoopRecoveryActive());

        bool observed_gap = false;
        for (unsigned ms = 0; ms < 3000 && !strstr(fake_tx, "ok\r\n"); ++ms) {
            for (unsigned tick = 0; tick < 50; ++tick) MotionService_OnStepTickFromISR();
            FakeRobot_AdvanceUs(1000);
            fake_motor_deg[2] = 8.0f - (float)fake_steps[2] * (360.0f / 3200.0f);
            MotionService_UpdateClosedLoop();
            AppSerial_Service();
            if (fake_steps[2] && MotionService_IsIdle() && !MotionService_IsClosedLoopRecoveryActive()
                && fake_motor_deg[2] > CL_DEADBAND_DEG) {
                observed_gap = true;
                assert(!strstr(fake_tx, "ok\r\n"));
            }
        }
        ClosedLoopStatus_t status;
        MotionService_GetClosedLoopStatus(&status);
        assert(observed_gap && status.accepted_rounds[2] >= 2);
        assert(strstr(fake_tx, "ok\r\n") && MotionService_HasClosedLoopSettled());
    } else {
        Send("G1 X5 Y190 Z235 F120");assert(CommandService_IsMotionPending());
        if(!strcmp(argv[1],"burst")) {
            Send("M400");assert(strstr(fake_tx,"error: previous command pending"));
            Send("M114");assert(strstr(fake_tx,"M114 PLAN"));
        }else assert(!strcmp(argv[1],"ordered"));
        FakeHardware_ClearTx();
        for(unsigned i=0;i<1000 && !strstr(fake_tx,"ok\r\n");i++) {
            CommandService_Service();AppSerial_Service();
        }
        assert(strstr(fake_tx,"ok\r\n")); /* G1受理回复，不是运动完成。 */
        Send("M400");assert(!strstr(fake_tx,"error:"));
        for(unsigned i=0;i<20000 && !strstr(fake_tx,"ok\r\n");i++) {
            CommandService_Service();
            for(unsigned j=0;j<100;j++)MotionService_OnStepTickFromISR();
            FakeRobot_AdvanceUs(2000);
            if (MotionService_IsIdle() && !CommandService_IsMotionPending()) {
                ClosedLoopStatus_t status;
                MotionService_GetClosedLoopStatus(&status);
                memcpy(fake_motor_deg, status.target_motor_deg, sizeof(fake_motor_deg));
            }
            MotionService_UpdateClosedLoop();
            AppSerial_Service();
        }
        assert(strstr(fake_tx,"ok\r\n") && MotionService_IsIdle());
        Send("M114");assert(strstr(fake_tx,"M114 PLAN X:5.00 Y:190.00 Z:235.00"));
    }
    printf("serial_reply_flow %s passed\n",argv[1]);return 0;
}
