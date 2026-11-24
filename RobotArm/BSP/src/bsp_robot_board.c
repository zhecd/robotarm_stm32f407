/**
 * @file bsp_robot_board.c
 * @brief 板级绑定：集中管理驱动串口、夹爪 PWM、限位引脚和 STEP 定时器。
 */
#include "bsp/bsp_robot_board.h"
#include "bsp_driver_config.h"
#include "driver/drv_servo.h"
#include "driver/drv_stepper.h"
#include "driver/drv_tmc2209.h"
#include "tim.h"
#include "usart.h"
#include "main.h"
#include <stdio.h>

void BSP_RobotBoard_StartStepTimer(void) { (void)HAL_TIM_Base_Start_IT(&htim6); }

bool BSP_RobotBoard_ReadDriverRegister(uint8_t node, uint8_t reg, uint32_t *out)
{
    return Drv_TMC2209_ReadRegister(&huart6,node,reg,out);
}
uint8_t BSP_RobotBoard_ReadDriverRegisterResult(uint8_t node, uint8_t reg, uint32_t *out)
{
    return (uint8_t)Drv_TMC2209_ReadRegisterResult(&huart6,node,reg,out);
}

bool BSP_RobotBoard_IsLimitTriggered(uint8_t joint)
{
    switch (joint) {
    case 0: return HAL_GPIO_ReadPin(M1_STOP_GPIO_Port, M1_STOP_Pin)==GPIO_PIN_RESET;
    case 1: return HAL_GPIO_ReadPin(M2_STOP_GPIO_Port, M2_STOP_Pin)==GPIO_PIN_RESET;
    case 2: return HAL_GPIO_ReadPin(M3_STOP_GPIO_Port, M3_STOP_Pin)==GPIO_PIN_RESET;
    default: return false;
    }
}

void BSP_RobotBoard_ConfigureMotionDrivers(void)
{
    for (uint8_t node = 0U; node < 3U; ++node) {
        Drv_TMC2209_ConfigNode(&huart6, node, TMC_DEFAULT_MICROSTEPS,
                              TMC_DEFAULT_IRUN, TMC_DEFAULT_IHOLD);
    }
}

void BSP_RobotBoard_InitGripper(void)
{
    (void)BSP_RobotBoard_TryInitGripper();
}

bool BSP_RobotBoard_TryInitGripper(void)
{
#if ROBOTARM_STARTUP_GRIPPER_IDLE
    /* 视觉标定期间只绑定通道，启动和 M999 均不主动输出夹爪 PWM。 */
    return Drv_Servo_Bind(Drv_Servo_GetHandle(), &htim2, TIM_CHANNEL_2);
#else
    return Drv_Servo_TryInit(Drv_Servo_GetHandle(), &htim2, TIM_CHANNEL_2);
#endif
}

bool BSP_RobotBoard_PrepareMotionDrivers(uint8_t *failed_node)
{
    if (failed_node!=NULL) *failed_node=0xFFU;
    (void)Drv_Stepper_EnableAll(false);
    for (uint8_t node=0U;node<3U;node++) {
        TmcConfigReport_t report={0};
        if (!Drv_TMC2209_ConfigNodeVerified(&huart6,node,TMC_DEFAULT_MICROSTEPS,
                                          TMC_DEFAULT_IRUN,TMC_DEFAULT_IHOLD,&report)) {
            if (failed_node!=NULL) *failed_node=node;
            /* 主循环失败后才输出一次，不在UART收发或临界区中打印。
             * 保留底层真实失败阶段；不增加读写、不重试、不启用执行器。
             * RESULT/READ为drv_tmc2209.h中的枚举，EXPECTED/OBSERVED按失败阶段解释，
             * 不能把“配置未确认”一概写成串口无应答或硬件损坏。 */
            printf("# TMC_CONFIG NODE:%u RESULT:%u REG:%02X READ:%u EXPECTED:%08lX OBSERVED:%08lX IFCNT:%u,%u GCONF:%08lX CHOPCONF:%08lX\r\n",
                   (unsigned)node,(unsigned)report.result,(unsigned)report.failed_register,
                   (unsigned)report.read_error,(unsigned long)report.expected,(unsigned long)report.observed,
                   (unsigned)report.ifcnt_before,(unsigned)report.ifcnt_after,
                   (unsigned long)report.gconf_observed,(unsigned long)report.chopconf_observed);
            return false;
        }
        /* 静音在使能前配置一次；不在手柄10ms轮询中阻塞UART或反复切换模式。 */
        if(!Drv_TMC2209_ConfigureQuietVerified(&huart6,node,&report)) {
            if(failed_node!=NULL)*failed_node=node;
            printf("# TMC_QUIET NODE:%u RESULT:%u REG:%02X READ:%u EXPECTED:%08lX OBSERVED:%08lX IFCNT:%u,%u\r\n",
                (unsigned)node,(unsigned)report.result,(unsigned)report.failed_register,
                (unsigned)report.read_error,(unsigned long)report.expected,(unsigned long)report.observed,
                (unsigned)report.ifcnt_before,(unsigned)report.ifcnt_after);
            return false;
        }
        printf("# TMC_QUIET NODE:%u CONFIGURED:1 GCONF:%08lX PWMCONF:%08lX SPREAD:%u TPWMTHRS_WRITE:0\r\n",
            (unsigned)node,(unsigned long)report.gconf_observed,(unsigned long)report.pwmconf_observed,
            (unsigned)((report.ioin_observed>>8)&1U));
    }
    return true;
}

void BSP_RobotBoard_StopActuators(void)
{
    /* EXTI可早于MX_TIM初始化；未绑定Instance时不得解引用定时器寄存器。 */
    if (htim6.Instance==TIM6) (void)HAL_TIM_Base_Stop_IT(&htim6);
    if (htim2.Instance==TIM2) {
        bool stopped=HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2)==HAL_OK;
        /* 安全停止绕过Device，必须同步驱动记录，避免M410继续报告旧PWM请求。 */
        Drv_Servo_RecordStopResult(Drv_Servo_GetHandle(),stopped);
    }
    (void)Drv_Stepper_EnableAll(false);
    (void)Drv_Stepper_WriteStepLevel(Drv_Stepper_GetM1(),false);
    (void)Drv_Stepper_WriteStepLevel(Drv_Stepper_GetM2(),false);
    (void)Drv_Stepper_WriteStepLevel(Drv_Stepper_GetM3(),false);
}
