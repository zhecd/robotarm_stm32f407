/**
 * @file bsp_robot_board.h
 * @brief 板级绑定：集中管理驱动串口、夹爪 PWM、限位引脚和 STEP 定时器。
 */
#ifndef ROBOTARM_BSP_ROBOT_BOARD_H
#define ROBOTARM_BSP_ROBOT_BOARD_H
#include <stdbool.h>
#include <stdint.h>

void BSP_RobotBoard_ConfigureMotionDrivers(void);
/** 禁用EN后逐节点确认配置；首错停止，failed_node成功为0xFF，失败为0..2。
 * 主循环独占USART6调用；不启用定时器、夹爪或电机，不替代机械回零。
 */
bool BSP_RobotBoard_PrepareMotionDrivers(uint8_t *failed_node);
void BSP_RobotBoard_InitGripper(void);
/** 正常启动时执行原闭合动作，传回PWM启动结果；诊断不能调用。 */
bool BSP_RobotBoard_TryInitGripper(void);
/** 停STEP定时器/PWM、禁用EN并拉低三路STEP；可在ISR及定时器初始化前调用。 */
void BSP_RobotBoard_StopActuators(void);
void BSP_RobotBoard_StartStepTimer(void);
bool BSP_RobotBoard_IsLimitTriggered(uint8_t joint_index);
/** 诊断限流/过载停止：关闭单轴IRQ及边沿检测、排空pending；不改变GPIO读取或共享NVIC。 */
void BSP_RobotBoard_MaskLimitInterrupt(uint8_t axis);
/** 只恢复当前读高的单轴；调用者负责短临界区及上层故障锁存检查。 */
bool BSP_RobotBoard_TryRearmLimitInterrupt(uint8_t axis);
/** 只分发已使能且pending的限位；已屏蔽轴只排空pending、不回调；无关位不清。 */
void BSP_RobotBoard_DispatchLimitInterrupts(void);
/** 仅诊断初始化：临界区内重置服务后，清pending、恢复原下降沿并重新使能三路。 */
void BSP_RobotBoard_ResetLimitInterrupts(void);
/** 读TMC总线原始节点寄存器；不假定节点地址与机械轴对应。 */
bool BSP_RobotBoard_ReadDriverRegister(uint8_t node, uint8_t reg, uint32_t *out);
/** 同一只读通路的详细结果：0成功，1参数，2接收关闭，3发送，4超时，5接收，6帧，7CRC。 */
uint8_t BSP_RobotBoard_ReadDriverRegisterResult(uint8_t node, uint8_t reg, uint32_t *out);

#endif
