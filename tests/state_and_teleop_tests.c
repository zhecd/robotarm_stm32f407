#include "app/app_teleop.h"
#include "state_service.h"
#include "command_service.h"
#include "motion_service.h"
#include "safety_service.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include "robot_config.h"
#include <assert.h>
#include <stdio.h>
static void Poll(void) { FakeRobot_AdvanceUs(PS2_POLL_INTERVAL_MS*1000U); App_Teleop_Task(); }
int main(void)
{
    FakeRobot_Reset(); FakeHardware_Reset(); StateService_Init();
    StateService_PublishAxisSample(0,10,20);
    StateServiceStatus_t a,b; StateService_GetStatus(&a);
    assert(a.generation==1 && a.encoder_valid[0] && a.motor_angle_deg[0]==10);
    StateService_PublishAxisReadFailure(0,2); StateService_GetStatus(&b);
    assert(b.generation==2 && !b.encoder_valid[0] && b.encoder_fail_count[0]==2);
    assert(a.encoder_valid[0]); /* 已取出的快照不会随内部状态改变。 */
    StateService_PublishAxisSample(9,0,0); StateService_GetStatus(&a);
    assert(a.generation==b.generation); StateService_GetStatus(NULL);
    SafetyService_Init(); MotionService_Init();
    assert(CommandService_Init(HOMEPOSE_X_MM,HOMEPOSE_Y_MM,HOMEPOSE_Z_MM)==ERR_OK);
    SafetyService_MarkHomed(); SafetyService_ClearAfterSuccessfulHoming();
    App_Teleop_Init(); App_Teleop_SetMode(SYS_MODE_PS2); fake_ps2_valid=true;
    Poll(); /* 先确认回中和松键，不能以带按键的首帧接管控制。 */
    fake_ps2_input.cross_pressed=true; Poll(); Poll();
    assert(fake_gripper_close_count==1); /* 长按不会重复触发夹爪。 */
    fake_ps2_input.cross_pressed=false; Poll();
    fake_ps2_input.square_pressed=true; Poll(); assert(fake_gripper_open_count==1);
    fake_ps2_input.left_x=255; Poll(); assert(MotionService_GetQueueCount()>0);
    fake_ps2_valid=false; Poll(); assert(MotionService_GetQueueCount()==0);
    puts("state_and_teleop_tests: passed"); return 0;
}
