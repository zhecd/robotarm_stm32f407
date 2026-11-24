/** 真实协议+Service只读查询；七种启动/运行模式均不触发夹爪或关节。 */
#include "app/app_serial_protocol.h"
#include "safety_service.h"
#include "motion_service.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static void Query(const char *line)
{ FakeHardware_ClearTx();FakeHardware_SendLine(line);AppSerial_Service(); }
int main(int argc,char **argv)
{
    assert(argc==2);
    unsigned mode=(unsigned)strtoul(argv[1],NULL,10);
    assert(mode<=APP_SERIAL_OUTPUT_LOCKED);
    FakeRobot_Reset();FakeHardware_Reset();SafetyService_Init();MotionService_Init();
    AppSerial_Init((AppSerialMode_t)mode);
    Query("M115");assert(strstr(fake_tx,"GRIPPER_STATUS"));
    Query("M410");
    assert(strcmp(fake_tx,"M410 BOUND:0 TARGET:NONE ANGLE_DEG:0 PWM_REQUESTED:0 HOLD_MS:600 ELAPSED_MS:0 ERROR:0 POSITION:UNMEASURED\r\n")==0);
    fake_gripper_status=(GripperStatus_t){.bound=true,.target=GRIPPER_TARGET_OPEN,
        .target_angle_deg=100,.pwm_requested=true,.hold_ms=600,.elapsed_ms=23};
    GripperStatus_t before=fake_gripper_status;
    Query("M410");assert(strstr(fake_tx,"TARGET:OPEN ANGLE_DEG:100 PWM_REQUESTED:1"));
    assert(memcmp(&before,&fake_gripper_status,sizeof(before))==0);
    fake_gripper_status.pwm_requested=false;fake_gripper_status.elapsed_ms=600;
    Query("M410");assert(strstr(fake_tx,"PWM_REQUESTED:0 HOLD_MS:600 ELAPSED_MS:600 ERROR:0"));
    fake_gripper_status.pwm_requested=true;fake_gripper_status.error=GRIPPER_ERROR_STOP;
    Query("M410");assert(strstr(fake_tx,"PWM_REQUESTED:1 HOLD_MS:600 ELAPSED_MS:600 ERROR:2"));
    Query("M410 X1");assert(strstr(fake_tx,"error:"));
    assert(!fake_joints_enabled && !fake_enable_true_count);
    assert(!fake_steps[0] && !fake_steps[1] && !fake_steps[2]);
    assert(!fake_gripper_open_count && !fake_gripper_close_count && !fake_gripper_init_count);
    assert(!fake_gripper_init_attempts && !fake_timer_start_count);
    assert(!FakeRobot_CriticalDepth());
    puts("gripper_protocol: query is read-only in this mode");return 0;
}
