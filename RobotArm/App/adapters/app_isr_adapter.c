/**
 * @file app_isr_adapter.c
 * @brief 中断适配：将 HAL 定时器和 GPIO 参数转换为服务事件。
 */
#include "app_isr_adapter.h"
#include "app_hardware_adapter.h"

#include "main.h"

#include "app/app_runtime_events.h"
#include "motion_service.h"
#include "waveform_service.h"
#include "limit_observation_service.h"
#include "limit_irq_guard.h"
#if ROBOTARM_LIMIT_STATE_MODE && !ROBOTARM_SEQUENTIAL_HOMING
#error State-sampled limits require sequential homing and its periodic safety checks
#endif
#if ROBOTARM_LIMIT_STATE_MODE
#include "common/limit_state_filter.h"
#include "platform_critical.h"
#include "platform_time.h"
#include "safety_service.h"
#endif
#if ROBOTARM_SEQUENTIAL_HOMING
#include "service/svc_homing_sequence.h"
static volatile bool s_limit_indicators_ready;
#endif
#if ROBOTARM_LIMIT_SEEK_TRIAL
/* 独立接触试验的短ISR钩子；正常固件不链接此策略。 */
void LimitSeek_ContactFromISR(uint8_t axis);
#endif
#if ROBOTARM_WAVEFORM_MODE
#include "bsp/bsp_limit_trace.h"
#endif

void App_ISR_OnLimitVectorEnter(void)
{
#if ROBOTARM_WAVEFORM_MODE
    BSP_LimitTrace_CaptureVector();
#endif
}

void App_ISR_OnLimitVectorExit(void) { /* IRQ2专用链接包装器在这里发布记录。 */ }
#if ROBOTARM_LIMIT_STATE_MODE
static LimitStateFilter s_runtime_limits;
/** 回零阶段使用逐轴状态机中的接触/释放消抖；READY后由独立纯算法确认。
 * 首样本立即暂停STEP，稳定低10ms才锁定LIMIT故障；稳定高10ms取消短脉冲。
 * 等待时保留EN保持力，确认故障时切断全部输出。TIM6发步前也做门禁。
 * 每路期限从第一份低样本开始，不因反复采样而延长，也不在ISR延时。 */
static bool StopForRuntimeLimit(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    if(!Svc_HomingSequence_RuntimeOwnsLimits()){
        LimitStateFilter_Reset(&s_runtime_limits);PlatformCritical_Exit(key);return false;
    }
    uint8_t low=0;
    for(uint8_t a=0;a<3U;a++)if(AppHardware_IsLimitTriggered(a))low|=(uint8_t)(1U<<a);
    LimitStateFilter_Sample(&s_runtime_limits,PlatformTime_NowMs(),low);
    bool blocked=LimitStateFilter_BlocksSteps(&s_runtime_limits);
    if(blocked && !AppHardware_StopJointSteps()){
        AppHardware_StopActuators();
        MotionService_AbortForFault(MOTION_FAULT_INPUT_LOST);SafetyService_ReportInternalFailure();
        PlatformCritical_Exit(key);return true;
    }
    if(s_runtime_limits.pressed || s_runtime_limits.unstable){
        AppHardware_StopActuators();
        if(s_runtime_limits.pressed){
            MotionService_AbortForFault(MOTION_FAULT_LIMIT_SWITCH);SafetyService_ReportLimitSwitch();
        }else{
            MotionService_AbortForFault(MOTION_FAULT_INPUT_LOST);SafetyService_ReportInputLost();
        }
    }
    PlatformCritical_Exit(key);return blocked;
}
#endif
void App_ISR_OnLimitsConfigured(void)
{
#if ROBOTARM_SEQUENTIAL_HOMING
    Svc_HomingSequence_OnLimitsConfigured();
    s_limit_indicators_ready=true;
#endif
}
void App_ISR_OnMillisecond(void)
{
#if ROBOTARM_SEQUENTIAL_HOMING
    /* SysTick早于MX_GPIO_Init启动，必须等限位GPIO配置完成才读取并写灯。
     * 故障/串口阻塞/回零期间同样显示，灯的原始电平不参与10ms安全消抖。 */
    if(s_limit_indicators_ready)AppHardware_UpdateLimitIndicators();
    Svc_HomingSequence_OnTickFromISR();
#endif
#if ROBOTARM_LIMIT_STATE_MODE
    (void)StopForRuntimeLimit();
#endif
}

void App_ISR_OnTimerElapsed(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
#if ROBOTARM_LIMIT_STATE_MODE
        if(StopForRuntimeLimit())return;
#endif
#if ROBOTARM_WAVEFORM_MODE
        WaveformService_OnTickFromISR();
#elif !ROBOTARM_DIAGNOSTIC_MODE
        MotionService_OnStepTickFromISR();
#endif
    }
}

void App_ISR_OnGpioEdge(uint16_t pin)
{
    if (pin == M1_STOP_Pin || pin == M2_STOP_Pin || pin == M3_STOP_Pin) {
#if ROBOTARM_LIMIT_STATE_MODE
        /* GPIO在BSP中已配置普通输入；即使误调用旧回调也不按次数停机。
         * 状态保护始终由1ms钩子和发步前门禁承担，按键回调仍按原路径处理。 */
        return;
#else
#if !ROBOTARM_DIAGNOSTIC_MODE
        uint8_t guard_axis=pin==M1_STOP_Pin?0U:pin==M2_STOP_Pin?1U:2U;
        if (LimitIrqGuard_OnEdgeFromISR(guard_axis)) {
            AppHardware_MaskLimitInterrupt(guard_axis);
            AppHardware_StopActuators();
            return;
        }
#if ROBOTARM_SEQUENTIAL_HOMING
        if(Svc_HomingSequence_OnLimitFromISR(guard_axis))return;
#endif
#if ROBOTARM_LIMIT_SEEK_TRIAL
        /* 已有过载/第256次回调先走上面的停机，接触过滤不得抢先豁免。 */
        LimitSeek_ContactFromISR(guard_axis);
#endif
#endif
#if ROBOTARM_WAVEFORM_MODE
        uint8_t axis=pin==M1_STOP_Pin?0U:pin==M2_STOP_Pin?1U:2U;
        WaveformService_OnLimitFromISR(axis);
#elif ROBOTARM_DIAGNOSTIC_MODE
        uint8_t axis=pin==M1_STOP_Pin?0U:pin==M2_STOP_Pin?1U:2U;
        if (LimitObservation_OnEdgeFromISR(axis))
            AppHardware_MaskDiagnosticLimitInterrupt(axis);
#else
        uint8_t mask = pin == M1_STOP_Pin ? MOTION_LIMIT_M1 :
                       pin == M2_STOP_Pin ? MOTION_LIMIT_M2 : MOTION_LIMIT_M3;
        MotionService_NotifyLimitSwitch(mask);
#endif
#endif
    } else if (pin == KEY_MODE_Pin) {
        AppRuntime_RequestModeSwitch();
    }
}
