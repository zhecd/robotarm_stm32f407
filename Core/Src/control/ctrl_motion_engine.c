/**
 * @file    ctrl_motion_engine.c
 * @brief   Bresenham motion engine implementation. / Bresenham 运动引擎实现。
 * @ingroup control
 */

#include "control/ctrl_motion_engine.h"
#include "robot_config.h"
#include <stdio.h>
#include <stdlib.h>

static MotionFrame_t s_buffer[RING_BUFFER_SIZE];
static volatile uint16_t s_head = 0;
static volatile uint16_t s_tail = 0;

static bool          s_running      = false;
static MotionFrame_t s_cur_frame;
static uint32_t      s_cur_tick     = 0;
static uint32_t      s_acc_m1       = 0;
static uint32_t      s_acc_m2       = 0;
static uint32_t      s_acc_m3       = 0;
static uint32_t      s_abs_m1       = 0;
static uint32_t      s_abs_m2       = 0;
static uint32_t      s_abs_m3       = 0;

static int32_t s_theory_m1 = 0;
static int32_t s_theory_m2 = 0;
static int32_t s_theory_m3 = 0;

void Ctrl_MotionEngine_Init(void)
{
    s_head = 0;
    s_tail = 0;
    s_running  = false;
    s_cur_tick = 0;
    s_theory_m1 = 0;
    s_theory_m2 = 0;
    s_theory_m3 = 0;
}

bool Ctrl_MotionEngine_PushFrame(const MotionFrame_t *frame)
{
    if (!frame) return false;
    uint16_t next = (s_head + 1) % RING_BUFFER_SIZE;
    if (next == s_tail) return false;
    s_buffer[s_head] = *frame;
    s_head = next;
    return true;
}

static bool PopFrame(MotionFrame_t *out)
{
    if (!out || s_head == s_tail) return false;
    *out = s_buffer[s_tail];
    s_tail = (s_tail + 1) % RING_BUFFER_SIZE;
    return true;
}

void Ctrl_MotionEngine_Clear(void)
{
    __disable_irq();
    s_tail = s_head;
    __enable_irq();
}

bool Ctrl_MotionEngine_IsRunning(void)
{
    return s_running;
}

uint16_t Ctrl_MotionEngine_GetQueueCount(void)
{
    if (s_head >= s_tail)
        return s_head - s_tail;
    else
        return RING_BUFFER_SIZE - s_tail + s_head;
}

void Ctrl_MotionEngine_GetTheorySteps(int32_t *m1, int32_t *m2, int32_t *m3)
{
    if (m1) *m1 = s_theory_m1;
    if (m2) *m2 = s_theory_m2;
    if (m3) *m3 = s_theory_m3;
}

void Ctrl_MotionEngine_ResetTheorySteps(void)
{
    __disable_irq();
    s_theory_m1 = 0;
    s_theory_m2 = 0;
    s_theory_m3 = 0;
    __enable_irq();
}

void Ctrl_MotionEngine_AdjustTheorySteps(int32_t dm1, int32_t dm2, int32_t dm3)
{
    __disable_irq();
    s_theory_m1 += dm1;
    s_theory_m2 += dm2;
    s_theory_m3 += dm3;
    __enable_irq();
}

/* ── TIM6 ISR: Bresenham step generation / TIM6 中断: Bresenham 步进生成 ── */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) return;

    if (!s_running) {
        if (!PopFrame(&s_cur_frame)) return;
        s_running  = true;
        s_cur_tick = 0;

        BSP_Stepper_SetDir(BSP_Stepper_GetM1(),
            (s_cur_frame.delta_m1 >= 0) ? STEPPER_DIR_CW : STEPPER_DIR_CCW);
        BSP_Stepper_SetDir(BSP_Stepper_GetM2(),
            (s_cur_frame.delta_m2 >= 0) ? STEPPER_DIR_CW : STEPPER_DIR_CCW);
        BSP_Stepper_SetDir(BSP_Stepper_GetM3(),
            (s_cur_frame.delta_m3 >= 0) ? STEPPER_DIR_CW : STEPPER_DIR_CCW);

        s_abs_m1 = (uint32_t)labs(s_cur_frame.delta_m1);
        s_abs_m2 = (uint32_t)labs(s_cur_frame.delta_m2);
        s_abs_m3 = (uint32_t)labs(s_cur_frame.delta_m3);

        s_acc_m1 = s_cur_frame.total_ticks / 2;
        s_acc_m2 = s_cur_frame.total_ticks / 2;
        s_acc_m3 = s_cur_frame.total_ticks / 2;
    }

    s_acc_m1 += s_abs_m1;
    if (s_acc_m1 >= s_cur_frame.total_ticks) {
        BSP_Stepper_Step(BSP_Stepper_GetM1());
        s_acc_m1 -= s_cur_frame.total_ticks;
    }

    s_acc_m2 += s_abs_m2;
    if (s_acc_m2 >= s_cur_frame.total_ticks) {
        BSP_Stepper_Step(BSP_Stepper_GetM2());
        s_acc_m2 -= s_cur_frame.total_ticks;
    }

    s_acc_m3 += s_abs_m3;
    if (s_acc_m3 >= s_cur_frame.total_ticks) {
        BSP_Stepper_Step(BSP_Stepper_GetM3());
        s_acc_m3 -= s_cur_frame.total_ticks;
    }

    if (++s_cur_tick >= s_cur_frame.total_ticks)
        s_running = false;
}
