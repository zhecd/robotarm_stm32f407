/**
 * @file app_serial_protocol.h
 * @brief 串口协议会话：命令解析、接收确认、完成等待、查询和停止。
 * @details ok/queued 表示接收；M400 表示软件运动队列完成，不证明夹爪或末端物理到位。
 */
#ifndef ROBOTARM_APP_SERIAL_PROTOCOL_H
#define ROBOTARM_APP_SERIAL_PROTOCOL_H
#include <stdbool.h>
typedef enum {
    APP_SERIAL_HOMING = 0,
    APP_SERIAL_READY,
    APP_SERIAL_FAILED,
    APP_SERIAL_DIAGNOSTIC,
    APP_SERIAL_WAVEFORM,
    APP_SERIAL_CHECKING,
    APP_SERIAL_OUTPUT_LOCKED /* 审计构建到达动作边界；未回零，不能当READY。 */
} AppSerialMode_t;
void AppSerial_Init(AppSerialMode_t mode);
void AppSerial_SetMode(AppSerialMode_t mode);
void AppSerial_Service(void);
#if defined(ROBOTARM_JOINT_RETURN_TEST) && ROBOTARM_JOINT_RETURN_TEST
/** 锁住物理手柄的模式切换，直到关节测试回到原位或故障回零。 */
bool AppSerial_JointTrialActive(void);
#endif
/** M999 只提出恢复请求；AppRuntime 负责启动和完成回零流程。 */
bool AppSerial_TakeRecoveryRequest(void);
/** 消费 M112 事件；已有故障时重复停止仍必须能取消本轮启动/恢复。 */
bool AppSerial_TakeStopRequest(void);
#endif
