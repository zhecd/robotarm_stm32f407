/** @file dev_joint.c @brief Joint-level adapter for motors and encoders. */
#include "device/dev_joint.h"

#include "driver/drv_as5600.h"
#include "driver/drv_stepper.h"
#include "driver/drv_tmc2209.h"
#include "robot_config.h"
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

void Dev_Joint_ConfigureDrivers(UART_HandleTypeDef *driver_uart)
{
    if (driver_uart == NULL) return;
    Drv_TMC2209_ConfigNode(driver_uart, 0U, TMC_DEFAULT_MICROSTEPS,
                           TMC_DEFAULT_IRUN, TMC_DEFAULT_IHOLD);
    Drv_TMC2209_ConfigNode(driver_uart, 1U, TMC_DEFAULT_MICROSTEPS,
                           TMC_DEFAULT_IRUN, TMC_DEFAULT_IHOLD);
    Drv_TMC2209_ConfigNode(driver_uart, 2U, TMC_DEFAULT_MICROSTEPS,
                           TMC_DEFAULT_IRUN, TMC_DEFAULT_IHOLD);
}

void Dev_Joint_EnableAll(bool enable)
{
    for (DevJointId_t joint = DEV_JOINT_M1; joint < DEV_JOINT_COUNT; joint++) {
        StepperDevice_t *stepper = GetStepper(joint);
        if (stepper != NULL) (void)Drv_Stepper_Enable(stepper, enable);
    }
}

bool Dev_Joint_SetDirection(DevJointId_t joint, bool clockwise)
{
    StepperDevice_t *stepper = GetStepper(joint);
    if (stepper == NULL) return false;
    return Drv_Stepper_SetDir(stepper,
        clockwise ? STEPPER_DIRECTION_CW : STEPPER_DIRECTION_CCW);
}

bool Dev_Joint_Step(DevJointId_t joint)
{
    StepperDevice_t *stepper = GetStepper(joint);
    return (stepper != NULL) && Drv_Stepper_Step(stepper);
}

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
