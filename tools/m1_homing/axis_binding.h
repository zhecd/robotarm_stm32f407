/** @file axis_binding.h
 * @brief 单轴验证的板级描述与输出计划；无HAL，不能自行写GPIO或启动动作。
 * 历史目录名保留m1_homing，逻辑和参考模块共用，轴差异只在本模块选择。
 */
#ifndef ROBOTARM_AXIS_BINDING_H
#define ROBOTARM_AXIS_BINDING_H
#include "m1_logic.h"

#ifndef SINGLE_AXIS_ID
#define SINGLE_AXIS_ID 1
#endif
#if SINGLE_AXIS_ID < 1 || SINGLE_AXIS_ID > 3
#error SINGLE_AXIS_ID must be 1, 2 or 3
#endif
#if SINGLE_AXIS_ID != 1 && (M1_JOG_MOTOR_TEN_DEG || M1_JOG_JOINT_TEN_DEG)
#error Larger M1 profiles are not permitted on M2 or M3
#endif

typedef struct {
    uint8_t id,index,uart_node,limit_bit;
    uint16_t step_a,step_b,dir_d;
} AxisBinding;

typedef struct {uint32_t a_bsrr,b_bsrr,d_bsrr;} AxisOutputs;
/** PCB10_1/当前Core引脚：M1 PA6/PD3/PC10，M2 PA7/PD4/PC11，M3 PB0/PD5/PC12。 */
const AxisBinding *AxisBinding_Get(unsigned axis_id);
M1Inputs AxisBinding_Inputs(unsigned axis_id,uint8_t limit_mask,bool feedback_valid,uint32_t main_seen_ms);
/** 只返回寄存器写计划。EN共用且低有效；未选轴STEP、夹爪PA1始终低。
 * 无效轴/空状态/无编译许可/截止锁存均禁用输出，不能因换轴绕过保护。
 */
AxisOutputs AxisBinding_Outputs(unsigned axis_id,const M1Test *motion,bool compile_permit,bool deadline_trip);
#endif
