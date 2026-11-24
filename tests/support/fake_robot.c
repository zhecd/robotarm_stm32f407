#include "fake_robot.h"
#include "device/dev_gripper.h"
#include "device/dev_limit_switch.h"
#include "platform_critical.h"
#include "platform_time.h"
#include "platform_delay.h"
#include "robot_home_pose.h"
#include <assert.h>
#include <string.h>

uint32_t fake_steps[DEV_JOINT_COUNT];
bool fake_clockwise[DEV_JOINT_COUNT];
bool fake_limits[DEV_JOINT_COUNT];
bool fake_step_failure[DEV_JOINT_COUNT];
bool fake_encoder_failure[DEV_JOINT_COUNT];
float fake_motor_deg[DEV_JOINT_COUNT];
bool fake_joints_enabled;
bool fake_wave_level[DEV_JOINT_COUNT];
uint32_t fake_wave_rises[DEV_JOINT_COUNT];
unsigned fake_enable_true_count;
unsigned fake_gripper_open_count;
unsigned fake_gripper_close_count;
unsigned fake_gripper_service_count;
bool fake_gripper_start_failure;
GripperStatus_t fake_gripper_status;
static uint64_t s_us;
static unsigned s_critical_depth;

void FakeRobot_Reset(void)
{
    memset(fake_steps, 0, sizeof(fake_steps));
    memset(fake_clockwise, 0, sizeof(fake_clockwise));
    memset(fake_limits, 0, sizeof(fake_limits));
    memset(fake_step_failure, 0, sizeof(fake_step_failure));
    memset(fake_encoder_failure, 0, sizeof(fake_encoder_failure));
    memset(fake_motor_deg, 0, sizeof(fake_motor_deg));
    fake_joints_enabled = false;
    memset(fake_wave_level,0,sizeof(fake_wave_level));
    memset(fake_wave_rises,0,sizeof(fake_wave_rises));
    fake_enable_true_count=0;
    fake_gripper_open_count = fake_gripper_close_count = 0;
    fake_gripper_service_count=0;
    fake_gripper_start_failure = false;
    fake_gripper_status=(GripperStatus_t){.hold_ms=600U};
    s_us = 0;
    assert(s_critical_depth == 0);
}
void FakeRobot_AdvanceUs(uint32_t us) { s_us += us; }
void FakeRobot_SetTimeUs(uint64_t us) { s_us = us; }
unsigned FakeRobot_CriticalDepth(void) { return s_critical_depth; }
void PlatformTime_Init(void) {}
uint32_t PlatformTime_NowMs(void) { return (uint32_t)(s_us / 1000U); }
uint32_t PlatformTime_NowUs(void) { return (uint32_t)s_us; }
void PlatformDelay_Ms(uint32_t ms) { s_us += (uint64_t)ms * 1000U; }
void PlatformDelay_Us(uint32_t us) { s_us += us; }
PlatformCriticalState_t PlatformCritical_Enter(void) { return s_critical_depth++; }
void PlatformCritical_Exit(PlatformCriticalState_t previous)
{
    assert(s_critical_depth == previous + 1U);
    s_critical_depth = previous;
}
void Dev_Joint_Init(void) {}
void Dev_Joint_ConfigureDrivers(void) {}
bool Dev_Joint_PrepareDrivers(uint8_t *failed_node)
{ if (failed_node) *failed_node=0xFFU;return true; }
void Dev_Joint_EnableAll(bool enabled)
{
    fake_joints_enabled = enabled;
    if (enabled) ++fake_enable_true_count;
}
bool Dev_Joint_WriteStepLevel(DevJointId_t axis, bool high)
{
    if ((unsigned)axis>=DEV_JOINT_COUNT || fake_step_failure[axis]) return false;
    if (high && !fake_wave_level[axis]) ++fake_wave_rises[axis];
    fake_wave_level[axis]=high; return true;
}
bool Dev_Joint_SetDirection(DevJointId_t axis, bool clockwise)
{
    if ((unsigned)axis >= DEV_JOINT_COUNT) return false;
    fake_clockwise[axis] = clockwise;
    return true;
}
bool Dev_Joint_SetDirectionLevel(DevJointId_t axis, bool high)
{
    /* 替身保留原clockwise记录；GPIO极性由另外的真实链路集成测试验收。 */
    return Dev_Joint_SetDirection(axis,!high);
}
bool Dev_Joint_Step(DevJointId_t axis)
{
    if ((unsigned)axis >= DEV_JOINT_COUNT || fake_step_failure[axis]) return false;
    fake_steps[axis]++;
    return true;
}
ErrorCode_t Dev_Joint_ReadMotorAngle(DevJointId_t axis, float *angle)
{
    if ((unsigned)axis >= DEV_JOINT_COUNT || !angle) return ERR_NULL_PARAM;
    if (fake_encoder_failure[axis]) return ERR_ENCODER_FAIL;
    *angle = fake_motor_deg[axis];
    return ERR_OK;
}
ErrorCode_t Dev_Joint_SetFeedbackZero(DevJointId_t axis)
{
    if ((unsigned)axis >= DEV_JOINT_COUNT) return ERR_NULL_PARAM;
    if (fake_encoder_failure[axis]) return ERR_ENCODER_FAIL;
    fake_motor_deg[axis] = 0;
    return ERR_OK;
}
bool Dev_Joint_IsWithinSoftLimit(DevJointId_t axis, float angle)
{
    float joint = RobotHomePose_MotorDegToJointDeg((RobotHomeAxis_t)axis, angle);
    return RobotHomePose_IsJointAngleSafe((RobotHomeAxis_t)axis, joint);
}
void Dev_Joint_PrintFeedbackStatus(void) {}
bool Dev_LimitSwitch_IsTriggered(DevJointId_t axis)
{
    return (unsigned)axis < DEV_JOINT_COUNT && fake_limits[axis];
}
void Dev_Gripper_Init(void) {}
bool Dev_Gripper_TryOpen(void)
{ if (fake_gripper_start_failure) return false;fake_gripper_open_count++;return true; }
bool Dev_Gripper_TryClose(void)
{ if (fake_gripper_start_failure) return false;fake_gripper_close_count++;return true; }
void Dev_Gripper_Open(void) { (void)Dev_Gripper_TryOpen(); }
void Dev_Gripper_Close(void) { (void)Dev_Gripper_TryClose(); }
void Dev_Gripper_Service(void) { ++fake_gripper_service_count; }
void Dev_Gripper_GetStatus(GripperStatus_t *out)
{ if (out) *out=fake_gripper_status; }
