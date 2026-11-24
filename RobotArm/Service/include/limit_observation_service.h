/**
 * @file limit_observation_service.h
 * @brief 只读限位观察：区分原始中断与经过时间确认的采样电平，不控制执行器。
 */
#ifndef ROBOTARM_LIMIT_OBSERVATION_SERVICE_H
#define ROBOTARM_LIMIT_OBSERVATION_SERVICE_H
#include <stdbool.h>
#include <stdint.h>
/* 每轴10ms窗口内256次回调即锁存诊断限流。SysTick被压制时窗口不前进，
 * 也仍会在第256次回调触发，避免依赖主循环或正常时钟才能终止中断占用。 */
#define LIMIT_OBSERVATION_IRQ_WINDOW_MS 10U
#define LIMIT_OBSERVATION_IRQ_BURST_LIMIT 256U
typedef struct {
    uint32_t edges[3];       /* 本次观察初始化后的EXTI回调数，非有效按压数。 */
    uint32_t presses[3];     /* 采样候选保持至少10ms后确认的按下次数。 */
    uint32_t sample_ms;      /* 最近一次主循环电平采样的毫秒时间。 */
    uint8_t raw_mask;        /* 最近采样触发位图，bit0/1/2对应M1/M2/M3。 */
    uint8_t stable_mask;     /* 已确认触发位图；候选变化期间保留旧值。 */
    uint8_t valid_mask;      /* 已完成首次10ms资格确认的轴位图。 */
    uint8_t irq_latch_mask;  /* 诊断回调过密锁存位图；该轴EDGE冻结，电平采样继续。 */
} LimitObservationStatus_t;
/** 主循环：清零统计；initial_mask为有效低电平位图，now_ms允许无符号回绕。 */
void LimitObservation_Init(uint8_t initial_mask, uint32_t now_ms);
/** 主循环：输入实测位图及时间；只确认采样结果，不证明采样间隙没有瞬态。 */
void LimitObservation_Sample(uint8_t triggered_mask, uint32_t now_ms);
/** EXTI：返回true要求适配层屏蔽该轴中断，直到诊断重新初始化。
 * 服务只决定锁存，不访问GPIO/IMR，不打印，不控制执行器；非法轴返回false。
 * 锁存后EDGE不再增长，不等于输入已恢复；PRESS和位图仍由主循环独立采样。
 */
bool LimitObservation_OnEdgeFromISR(uint8_t axis);
/** 主循环：临界区复制一致快照；out为空时忽略。 */
void LimitObservation_GetStatus(LimitObservationStatus_t *out);
#endif
