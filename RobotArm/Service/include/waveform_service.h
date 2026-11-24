/**
 * @file waveform_service.h
 * @brief 单轴有限 STEP 波形服务；始终禁用驱动，不用于电机动作。
 */
#ifndef ROBOTARM_WAVEFORM_SERVICE_H
#define ROBOTARM_WAVEFORM_SERVICE_H
#include <stdbool.h>
#include <stdint.h>
typedef enum {
    WAVE_IDLE, WAVE_RUNNING, WAVE_DONE, WAVE_HOST_STOP,
    WAVE_LIMIT, WAVE_HEARTBEAT, WAVE_ENCODER, WAVE_SERIAL, WAVE_OUTPUT, WAVE_IRQ_OVERLOAD
} WaveformState_t;
typedef struct {
    WaveformState_t state;
    uint32_t next_id; /* 本次启动内递增；已接受的请求不可重放。复位后从 1 开始。 */
    uint16_t emitted; /* 已产生的上升沿数量，不表示电机实际步数或到位。 */
    uint8_t axis;     /* 0=M1，1=M2，2=M3。 */
    uint8_t limit_mask; /* 导致本任务停止的逻辑轴位图：bit0/1/2=M1/M2/M3。 */
    uint8_t limit_raw_mask; /* 停止时同步读取的触发位图；瞬态边沿可能已恢复为0。 */
    uint8_t limit_source; /* 0=无，1=EXTI边沿，2=TIM6电平轮询。停止后保留。 */
    uint32_t limit_edges[3]; /* 本次启动以来各轴EXTI事件计数，饱和不回绕。 */
} WaveformStatus_t;
void WaveformService_Init(void);
/** 主循环：刷新运行心跳及三路编码器健康状态；失败立即停波。 */
void WaveformService_Feed(bool encoders_healthy);
/** 主循环：id 匹配 next_id；N=1..100，F 只接受 10/20/25/50/100 Hz。
 * dir_high 表示引脚电平，不代表机械正方向；无队列，忙时拒绝。
 */
bool WaveformService_Start(uint32_t id, uint8_t axis, bool dir_high,
                           uint16_t pulses, uint16_t frequency_hz);
void WaveformService_Stop(WaveformState_t reason);
void WaveformService_GetStatus(WaveformStatus_t *out);
/** TIM6 ISR 每 20 us 调用；不读 I2C、不打印。限位任一路触发即停止。 */
void WaveformService_OnTickFromISR(void);
/** EXTI入口传逻辑轴0..2；短暂边沿仍立即停波，同时保留来源供诊断。 */
void WaveformService_OnLimitFromISR(uint8_t axis);
#endif
