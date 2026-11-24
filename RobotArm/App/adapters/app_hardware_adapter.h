/**
 * @file app_hardware_adapter.h
 * @brief 应用硬件端口：将 BSP/Device 操作转换为应用可用的简单接口。
 */
#ifndef ROBOTARM_APP_HARDWARE_ADAPTER_H
#define ROBOTARM_APP_HARDWARE_ADAPTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t left_x;
    uint8_t left_y;
    uint8_t right_y;
    bool cross_pressed;
    bool square_pressed;
} AppTeleopInput_t;

/** 正常入口初始化；保持 STEP/PWM/EN 停止，夹爪初始化由就绪检查通过后显式调用。 */
void AppHardware_Init(void);
/** 可能立即闭合夹爪，仅正常入口在编码器就绪后调用一次。 */
void AppHardware_InitGripper(void);
/** 检查IRQ许可并传回夹爪初始化结果；失败由App决定停止启动。 */
bool AppHardware_TryInitGripper(void);
/** 仅初始化通信、编码器和限位读取；不启动电机、夹爪或回零。 */
void AppHardware_InitDiagnostics(void);
/** 诊断ISR/初始化专用，分别在最高优先级EXTI和Platform临界区中调用。 */
void AppHardware_MaskDiagnosticLimitInterrupt(uint8_t axis);
void AppHardware_ResetDiagnosticLimitInterrupts(void);
/** ISR/主循环均可调用；只屏蔽指定限位线/停止输出，不打印、不延时。 */
void AppHardware_MaskLimitInterrupt(uint8_t axis);
void AppHardware_StopActuators(void);
/** 仅把三路STEP拉低；消抖等待期间保留驱动保持力，不启用任何输出。 */
bool AppHardware_StopJointSteps(void);
bool AppHardware_ReadMotorAngle(uint8_t axis, float *out_deg);
typedef struct {
    uint32_t ioin, gconf, chopconf, ifcnt;
} AppDriverSnapshot_t;
/** 主循环、波形空闲时读取；失败不修改输出，不写TMC配置。地址不代表轴号。 */
bool AppHardware_ReadDriverSnapshot(uint8_t node, AppDriverSnapshot_t *out);
typedef struct {
    uint32_t ioin, drv_status;
    uint8_t error; /* 0成功；1参数、2接收关闭、3发送、4超时、5接收、6帧、7CRC。 */
    uint8_t failed_register; /* 失败寄存器；0xFF表示没有发出读请求或全部成功。 */
} AppDriverHealth_t;
/** 主循环空闲诊断：顺序读IOIN/DRV_STATUS，不清故障、不写电流。
 * 成功才发布完整原始值；失败清零原始值并报告error/failed_register。
 * CS_ACTUAL仅为0..31编码，不能在未知VREF/Rsense时换算安培。
 */
bool AppHardware_ReadDriverHealth(uint8_t node, AppDriverHealth_t *out);
void AppHardware_ConfigureMotionDrivers(void);
/** 正常启动/显式恢复用的确认端口；始终禁用EN，主循环独占驱动总线。
 * 阻塞UART期间保持中断开放；前后检查限位中断过载锁存，不触发机械动作。
 * failed_node成功为0xFF，节点失败为0..2，过载阻止准备时为0xFF。
 */
bool AppHardware_PrepareMotionDrivers(uint8_t *failed_node);
void AppHardware_EnableJoints(bool enabled);
/** 只记录LED1显示模式：G-code常亮，PS2每500ms翻转，不改变控制模式或动作许可。 */
void AppHardware_SetModeIndicator(bool gcode_on, bool ps2_on);
/** 有界GPIO操作：LED1模式指示，LED2/3/4分别显示三路限位原始低有效状态。
 * 正常固件GPIO配置完成后由1ms钩子调用，不含I2C、UART、延时或动作输出。 */
void AppHardware_UpdateLimitIndicators(void);
void AppHardware_StartStepTimer(void);

bool AppHardware_IsLimitTriggered(uint8_t joint_index);
bool AppHardware_ReadTeleopInput(AppTeleopInput_t *out_input);
/** 已收PS2诊断快照；查询不得轮询总线、重配手柄、清故障或启动动作。 */
typedef struct {
    uint32_t frames,valid_frames,invalid_frames,last_invalid_ms,first_invalid_ms;
    uint8_t last_status,config_phase,first_invalid_recorded,first_invalid_status;
    uint8_t last_frame[9],first_invalid_frame[9];
} AppTeleopDiagnostics_t;
void AppHardware_GetTeleopDiagnostics(AppTeleopDiagnostics_t *out);

void AppHardware_SendText(const char *text);
bool AppHardware_TakeLineTimeout(void);
bool AppHardware_TakeRxOverflow(void);
bool AppHardware_TakeTxOverflow(void);
bool AppHardware_ReadLine(char *out_line, size_t out_size);
void AppHardware_DiscardRx(void);

#endif /* ROBOTARM_APP_HARDWARE_ADAPTER_H */
