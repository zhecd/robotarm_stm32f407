#include "motor_core.h"
#include <stdio.h>
#include <stdlib.h>

static MotionBuffer_t s_motion_buffer;

static bool is_running = false;
static MotionFrame_t current_frame;
static uint32_t current_tick = 0;

static uint32_t acc_m1 = 0;
static uint32_t acc_m2 = 0;
static uint32_t acc_m3 = 0;

static uint32_t abs_steps_m1 = 0;
static uint32_t abs_steps_m2 = 0;
static uint32_t abs_steps_m3 = 0;

int32_t g_theory_steps_m1 = 0;
int32_t g_theory_steps_m2 = 0;
int32_t g_theory_steps_m3 = 0;

void Motor_Core_Init(void)
{
    s_motion_buffer.head = 0;
    s_motion_buffer.tail = 0;

    is_running = false;
    current_tick = 0;

    g_theory_steps_m1 = 0;
    g_theory_steps_m2 = 0;
    g_theory_steps_m3 = 0;
}

bool Motor_Buffer_Push(const MotionFrame_t *frame)
{
    if (frame == NULL) {
        return false;
    }
    
    uint16_t next_head = (s_motion_buffer.head + 1) % RING_BUFFER_SIZE;
    
    if (next_head == s_motion_buffer.tail) {
        return false; 
    }
    
    s_motion_buffer.frames[s_motion_buffer.head] = *frame;
    s_motion_buffer.head = next_head;
    
    return true;
}

bool Motor_Buffer_Pop(MotionFrame_t *out_frame)
{
    if (out_frame == NULL) {
        return false;
    }
    
    if (s_motion_buffer.head == s_motion_buffer.tail) {
        return false; 
    }
    
    *out_frame = s_motion_buffer.frames[s_motion_buffer.tail];
    s_motion_buffer.tail = (s_motion_buffer.tail + 1) % RING_BUFFER_SIZE;
    
    return true;
}

void Motor_Buffer_Clear(void)
{
    __disable_irq();
    s_motion_buffer.tail = s_motion_buffer.head;
    __enable_irq();
}

bool Motor_Core_IsRunning(void)
{
    return is_running;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) 
    {
        if (!is_running) 
        {
            if (Motor_Buffer_Pop(&current_frame))
            {
                is_running = true;
                current_tick = 0;

                BSP_Stepper_SetDir(&Motor_M1, (current_frame.delta_m1 >= 0) ? STEPPER_DIR_CW : STEPPER_DIR_CCW);
                BSP_Stepper_SetDir(&Motor_M2, (current_frame.delta_m2 >= 0) ? STEPPER_DIR_CW : STEPPER_DIR_CCW);
                BSP_Stepper_SetDir(&Motor_M3, (current_frame.delta_m3 >= 0) ? STEPPER_DIR_CW : STEPPER_DIR_CCW);
                
                abs_steps_m1 = abs(current_frame.delta_m1);
                abs_steps_m2 = abs(current_frame.delta_m2);
                abs_steps_m3 = abs(current_frame.delta_m3);
                
                acc_m1 = current_frame.total_ticks / 2;
                acc_m2 = current_frame.total_ticks / 2;
                acc_m3 = current_frame.total_ticks / 2;
            }
            else 
            {
                return;
            }
        }

        if (is_running) 
        {
            acc_m1 += abs_steps_m1;
            if (acc_m1 >= current_frame.total_ticks) {
                BSP_Stepper_Step(&Motor_M1);
                acc_m1 -= current_frame.total_ticks;
            }
            
            acc_m2 += abs_steps_m2;
            if (acc_m2 >= current_frame.total_ticks) {
                BSP_Stepper_Step(&Motor_M2);
                acc_m2 -= current_frame.total_ticks;
            }
            
            acc_m3 += abs_steps_m3;
            if (acc_m3 >= current_frame.total_ticks) {
                BSP_Stepper_Step(&Motor_M3);
                acc_m3 -= current_frame.total_ticks;
            }
            
            current_tick++;
            
            if (current_tick >= current_frame.total_ticks) {
                is_running = false;
            }
        }
    }
}

uint16_t Motor_Buffer_GetCount(void)
{
    if (s_motion_buffer.head >= s_motion_buffer.tail) {
        return s_motion_buffer.head - s_motion_buffer.tail;
    } else {
        return RING_BUFFER_SIZE - s_motion_buffer.tail + s_motion_buffer.head;
    }
}

void Motor_Core_GetTheorySteps(int32_t *m1, int32_t *m2, int32_t *m3)
{
    if (m1) *m1 = g_theory_steps_m1;
    if (m2) *m2 = g_theory_steps_m2;
    if (m3) *m3 = g_theory_steps_m3;
}

void Motor_Core_ResetTheorySteps(void)
{
    __disable_irq();
    g_theory_steps_m1 = 0;
    g_theory_steps_m2 = 0;
    g_theory_steps_m3 = 0;
    __enable_irq();
}

void Motor_Core_AdjustTheorySteps(int32_t dm1, int32_t dm2, int32_t dm3)
{
    __disable_irq();
    g_theory_steps_m1 += dm1;
    g_theory_steps_m2 += dm2;
    g_theory_steps_m3 += dm3;
    __enable_irq();
}
