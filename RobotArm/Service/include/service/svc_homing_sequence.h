/** @file svc_homing_sequence.h
 * @brief 正式回零编排：生产并行，历史逐轴对照；逻辑轴/快照不泄漏HAL类型。
 * @details 所有入口内部使用短临界区，不在ISR读I2C/UART或打印。
 * Production/TeachingGpio预设显式选用本服务；历史预设保留旧回零供对照。
 */
#ifndef ROBOTARM_SVC_HOMING_SEQUENCE_H
#define ROBOTARM_SVC_HOMING_SEQUENCE_H
#include "common/home_sequence.h"
typedef struct {HomeSequence flow;bool configured,committed,boot_ready;} SvcHomingSequenceStatus;
void Svc_HomingSequence_OnLimitsConfigured(void);
/** 由SysTick的1ms适配器调用；主循环不得通过快速重复调用加速发步。 */
void Svc_HomingSequence_OnTickFromISR(void);
bool Svc_HomingSequence_OnLimitFromISR(uint8_t axis);
bool Svc_HomingSequence_PrepareStartup(bool recovery);
/** 仅说明启动输入已稳定，不代表零位有效或允许动作。 */
bool Svc_HomingSequence_BootReady(void);
/** 按构建配置启动一次流程；准备失败返回false，不自行复位或清故障。 */
bool Svc_HomingSequence_Start(void);
void Svc_HomingSequence_MainHeartbeat(void);
void Svc_HomingSequence_Cancel(void);
bool Svc_HomingSequence_CommitReady(void);
/** 三轴释放/重新布防全部确认后，只保持EN、不发STEP，供标定前稳定。 */
bool Svc_HomingSequence_HoldForCalibration(void);
/** 仅供App尚未发布READY的交接窗口复查；已提交后新压限位必须收回成功。 */
bool Svc_HomingSequence_ValidateHandoff(void);
bool Svc_HomingSequence_HasFailure(void);
/** 标定交接已完成后，限位由正常运动保护接管；ISR只读，不改变许可。 */
bool Svc_HomingSequence_RuntimeOwnsLimits(void);
void Svc_HomingSequence_GetStatus(SvcHomingSequenceStatus *out);
#endif
