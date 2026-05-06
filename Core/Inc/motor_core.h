#ifndef __MOTOR_CORE_H__
#define __MOTOR_CORE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "bsp_stepper.h" // 👈 新增：引入底层物理动作的封装

#define RING_BUFFER_SIZE 64

// 单帧运动指令结构体
typedef struct {
    int32_t delta_m1;    // M1 轴步数增量 (带符号)
    int32_t delta_m2;    // M2 轴步数增量 (带符号)
    int32_t delta_m3;    // M3 轴步数增量 (带符号)
    uint32_t total_ticks;// 完成这一帧需要的定时器中断次数
} MotionFrame_t;

// 无锁环形缓冲区结构体
typedef struct {
    MotionFrame_t frames[RING_BUFFER_SIZE];
    volatile uint16_t head; // 写入头指针
    volatile uint16_t tail; // 读取尾指针
} MotionBuffer_t;

// 核心层 API
void Motor_Core_Init(void);
bool Motor_Buffer_Push(const MotionFrame_t *frame);
bool Motor_Buffer_Pop(MotionFrame_t *out_frame);
void Motor_Buffer_Clear(void);
bool Motor_Core_IsRunning(void);
uint16_t Motor_Buffer_GetCount(void);

// 理论步数追踪
void Motor_Core_GetTheorySteps(int32_t *m1, int32_t *m2, int32_t *m3);
void Motor_Core_ResetTheorySteps(void);
void Motor_Core_AdjustTheorySteps(int32_t dm1, int32_t dm2, int32_t dm3);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CORE_H__ */
