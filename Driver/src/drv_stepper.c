/**
 * @file    bsp_stepper.c
 * @brief   Stepper motor GPIO driver implementation / 濮濄儴绻橀悽鍨簚 GPIO 妞瑰崬濮╃€圭偟骞?
 * @ingroup bsp
 */

#include "driver/drv_stepper.h"

/* 閳光偓閳光偓 3 motor instances / 娑撳閲滈悽鍨簚鐎圭偘绶?閳光偓閳光偓 */

static StepperDevice_t s_motor_m1 = {
    .step_port        = M1_STEP_GPIO_Port,
    .step_pin         = M1_STEP_Pin,
    .dir_port         = M1_DIR_GPIO_Port,
    .dir_pin          = M1_DIR_Pin,
    .current_dir      = STEPPER_DIRECTION_CW,
    .absolute_position = 0
};

static StepperDevice_t s_motor_m2 = {
    .step_port        = M2_STEP_GPIO_Port,
    .step_pin         = M2_STEP_Pin,
    .dir_port         = M2_DIR_GPIO_Port,
    .dir_pin          = M2_DIR_Pin,
    .current_dir      = STEPPER_DIRECTION_CW,
    .absolute_position = 0
};

static StepperDevice_t s_motor_m3 = {
    .step_port        = M3_STEP_GPIO_Port,
    .step_pin         = M3_STEP_Pin,
    .dir_port         = M3_DIR_GPIO_Port,
    .dir_pin          = M3_DIR_Pin,
    .current_dir      = STEPPER_DIRECTION_CW,
    .absolute_position = 0
};

void Drv_Stepper_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* EN pin / 娴ｈ儻鍏樺鏇″壖 */
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
    gpio.Pin   = EN_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(EN_GPIO_Port, &gpio);

    /* M1 STEP + DIR / M1 濮濄儴绻?閺傜懓鎮?*/
    HAL_GPIO_WritePin(M1_STEP_GPIO_Port, M1_STEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, GPIO_PIN_RESET);
    gpio.Pin = M1_STEP_Pin;  HAL_GPIO_Init(M1_STEP_GPIO_Port, &gpio);
    gpio.Pin = M1_DIR_Pin;   HAL_GPIO_Init(M1_DIR_GPIO_Port, &gpio);

    /* M2 STEP + DIR / M2 濮濄儴绻?閺傜懓鎮?*/
    HAL_GPIO_WritePin(M2_STEP_GPIO_Port, M2_STEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, GPIO_PIN_RESET);
    gpio.Pin = M2_STEP_Pin;  HAL_GPIO_Init(M2_STEP_GPIO_Port, &gpio);
    gpio.Pin = M2_DIR_Pin;   HAL_GPIO_Init(M2_DIR_GPIO_Port, &gpio);

    /* M3 STEP + DIR / M3 濮濄儴绻?閺傜懓鎮?*/
    HAL_GPIO_WritePin(M3_STEP_GPIO_Port, M3_STEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_DIR_GPIO_Port, M3_DIR_Pin, GPIO_PIN_RESET);
    gpio.Pin = M3_STEP_Pin;  HAL_GPIO_Init(M3_STEP_GPIO_Port, &gpio);
    gpio.Pin = M3_DIR_Pin;   HAL_GPIO_Init(M3_DIR_GPIO_Port, &gpio);
}

bool Drv_Stepper_Enable(StepperDevice_t *motor, bool enable)
{
    if (!motor) return false;
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin,
                      enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
    return true;
}

bool Drv_Stepper_SetDir(StepperDevice_t *motor, StepperDirection_t dir)
{
    if (!motor) return false;
    motor->current_dir = dir;
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin,
                      (dir == STEPPER_DIRECTION_CW) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    return true;
}

bool Drv_Stepper_Step(StepperDevice_t *motor)
{
    if (!motor) return false;

    HAL_GPIO_WritePin(motor->step_port, motor->step_pin, GPIO_PIN_SET);
    __NOP(); __NOP(); __NOP(); __NOP();
    HAL_GPIO_WritePin(motor->step_port, motor->step_pin, GPIO_PIN_RESET);

    motor->absolute_position += (motor->current_dir == STEPPER_DIRECTION_CW) ? 1 : -1;
    return true;
}

/* 閳光偓閳光偓 Accessor functions / 鐠佸潡妫堕崳銊ュ毐閺?閳光偓閳光偓 */

StepperDevice_t *Drv_Stepper_GetM1(void) { return &s_motor_m1; }
StepperDevice_t *Drv_Stepper_GetM2(void) { return &s_motor_m2; }
StepperDevice_t *Drv_Stepper_GetM3(void) { return &s_motor_m3; }
