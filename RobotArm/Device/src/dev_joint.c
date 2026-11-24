/**
 * @file dev_joint.c
 * @brief 关节设备：按逻辑轴组织步进输出、编码器读取与关节角软限位判断。
 */
#include "device/dev_joint.h"

#include "driver/drv_as5600.h"
#include "driver/drv_stepper.h"
#include "driver/drv_tmc2209.h"
#include "bsp_driver_config.h"
#include "bsp/bsp_robot_board.h"
#include "robot_home_pose.h"

static StepperDevice_t *GetStepper(DevJointId_t joint)
{
    switch (joint) {
    case DEV_JOINT_M1: return Drv_Stepper_GetM1();
    case DEV_JOINT_M2: return Drv_Stepper_GetM2();
    case DEV_JOINT_M3: return Drv_Stepper_GetM3();
    default: return NULL;
    }
}

static As5600Device_t *GetEncoder(DevJointId_t joint)
{
    switch (joint) {
    case DEV_JOINT_M1: return Drv_AS5600_GetM1();
    case DEV_JOINT_M2: return Drv_AS5600_GetM2();
    case DEV_JOINT_M3: return Drv_AS5600_GetM3();
    default: return NULL;
    }
}

void Dev_Joint_Init(void)
{
    Drv_Stepper_Init();
    (void)Drv_AS5600_Init();
}

void Dev_Joint_ConfigureDrivers(void)
{
    BSP_RobotBoard_ConfigureMotionDrivers();
}

bool Dev_Joint_PrepareDrivers(uint8_t *failed_node)
{
    return BSP_RobotBoard_PrepareMotionDrivers(failed_node);
}

void Dev_Joint_EnableAll(bool enable)
{
    (void)Drv_Stepper_EnableAll(enable);
}

bool Dev_Joint_SetDirection(DevJointId_t joint, bool clockwise)
{
    StepperDevice_t *stepper = GetStepper(joint);
    if (stepper == NULL) return false;
    return Drv_Stepper_SetDir(stepper,
        clockwise ? STEPPER_DIRECTION_CW : STEPPER_DIRECTION_CCW);
}

bool Dev_Joint_SetDirectionLevel(DevJointId_t joint, bool high)
{
    StepperDevice_t *stepper = GetStepper(joint);
    if (stepper == NULL) return false;
    /* 兼容驱动CW枚举写低电平、CCW写高电平；只在设备边界完成此转换。
     * 不把high直接传给clockwise，否则DIR配置会反相。 */
    return Drv_Stepper_SetDir(stepper,
        high ? STEPPER_DIRECTION_CCW : STEPPER_DIRECTION_CW);
}

bool Dev_Joint_Step(DevJointId_t joint)
{
    StepperDevice_t *stepper = GetStepper(joint);
    return (stepper != NULL) && Drv_Stepper_Step(stepper);
}

bool Dev_Joint_WriteStepLevel(DevJointId_t joint, bool high)
{
    return Drv_Stepper_WriteStepLevel(GetStepper(joint),high);
}

/** 按逻辑轴读连续电机轴角度（deg），失败时调用者必须检查返回码。 */
ErrorCode_t Dev_Joint_ReadMotorAngle(DevJointId_t joint, float *angle_deg)
{
    As5600Device_t *encoder = GetEncoder(joint);
    if (encoder == NULL || angle_deg == NULL) return ERR_NULL_PARAM;
    ErrorCode_t status = Drv_AS5600_Update(encoder);
    if (status == ERR_OK) *angle_deg = encoder->angle_deg;
    return status;
}

ErrorCode_t Dev_Joint_SetFeedbackZero(DevJointId_t joint)
{
    As5600Device_t *encoder = GetEncoder(joint);
    return (encoder == NULL) ? ERR_NULL_PARAM : Drv_AS5600_SetZero(encoder);
}

/** 将电机轴角度转换为关节角后判断软限位，不直接比较两种角度。 */
bool Dev_Joint_IsWithinSoftLimit(DevJointId_t joint, float motor_angle_deg)
{
    if (joint >= DEV_JOINT_COUNT) return false;
    float joint_deg = RobotHomePose_MotorDegToJointDeg((RobotHomeAxis_t)joint,
                                                        motor_angle_deg);
    return RobotHomePose_IsJointAngleSafe((RobotHomeAxis_t)joint, joint_deg);
}

void Dev_Joint_PrintFeedbackStatus(void)
{
    Drv_AS5600_PrintStatus();
}
