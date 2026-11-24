/** 电脑测试替身：替换时间、临界区和设备，不替换被测的业务实现。 */
#ifndef ROBOTARM_FAKE_ROBOT_H
#define ROBOTARM_FAKE_ROBOT_H
#include <stdbool.h>
#include <stdint.h>
#include "device/dev_joint.h"
#include "gripper_status.h"

extern uint32_t fake_steps[DEV_JOINT_COUNT];
extern bool fake_clockwise[DEV_JOINT_COUNT];
extern bool fake_limits[DEV_JOINT_COUNT];
extern bool fake_step_failure[DEV_JOINT_COUNT];
extern bool fake_encoder_failure[DEV_JOINT_COUNT];
extern float fake_motor_deg[DEV_JOINT_COUNT];
extern bool fake_joints_enabled;
extern bool fake_wave_level[DEV_JOINT_COUNT];
extern uint32_t fake_wave_rises[DEV_JOINT_COUNT];
extern unsigned fake_enable_true_count;
extern unsigned fake_gripper_open_count;
extern unsigned fake_gripper_close_count;
extern unsigned fake_gripper_service_count;
extern bool fake_gripper_start_failure;
extern GripperStatus_t fake_gripper_status;
void FakeRobot_Reset(void);
void FakeRobot_AdvanceUs(uint32_t us);
void FakeRobot_SetTimeUs(uint64_t us);
unsigned FakeRobot_CriticalDepth(void);
#endif
