/**
 * @file dev_joint.h
 * @brief 关节设备：按逻辑轴组织步进输出、编码器读取与关节角软限位判断。
 */
#ifndef ROBOTARM_DEV_JOINT_H
#define ROBOTARM_DEV_JOINT_H

#include <stdbool.h>
#include <stdint.h>
#include "error_code.h"

typedef enum {
    DEV_JOINT_M1 = 0,
    DEV_JOINT_M2,
    DEV_JOINT_M3,
    DEV_JOINT_COUNT
} DevJointId_t;

void Dev_Joint_Init(void);
/** 配置三轴驱动器；具体串口由 BSP 绑定，不向服务层暴露 HAL 句柄。 */
void Dev_Joint_ConfigureDrivers(void);
/** 仅配置并确认三个驱动；不会开放运动许可，failed_node为总线地址。 */
bool Dev_Joint_PrepareDrivers(uint8_t *failed_node);
void Dev_Joint_EnableAll(bool enable);
bool Dev_Joint_SetDirection(DevJointId_t joint, bool clockwise);
/** 设置逻辑轴DIR电平：false=D0低、true=D1高，不等同于物理顺/逆时针。
 * 回零方向表使用本接口；兼容SetDirection的true=CW语义保持不变。
 * 只改变方向，不使能电机、不发STEP；失败不写GPIO。 */
bool Dev_Joint_SetDirectionLevel(DevJointId_t joint, bool high);
bool Dev_Joint_Step(DevJointId_t joint);
/** 波形验收专用：写 STEP 电平，不更新运动位置；调用者必须保持共用 EN 禁用。 */
bool Dev_Joint_WriteStepLevel(DevJointId_t joint, bool high);

/** 按逻辑轴读连续电机轴角度（deg），失败时调用者必须检查返回码。 */
ErrorCode_t Dev_Joint_ReadMotorAngle(DevJointId_t joint, float *angle_deg);
ErrorCode_t Dev_Joint_SetFeedbackZero(DevJointId_t joint);
/** 将电机轴角度转换为关节角后判断软限位，不直接比较两种角度。 */
bool Dev_Joint_IsWithinSoftLimit(DevJointId_t joint, float motor_angle_deg);
void Dev_Joint_PrintFeedbackStatus(void);

#endif /* ROBOTARM_DEV_JOINT_H */
