/**
 * @file drv_stepper.c
 * @brief STEP/DIR 驱动：方向、单次脉冲与三轴共用使能。
 * @details NOP 数量不等同于已验证的 STEP 脉宽，实际脉宽和 DIR 建立时间须后续实机测量。
 */

#include "driver/drv_stepper.h"

/* 3 motor instances */

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

    /* EN pin */
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
    gpio.Pin   = EN_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(EN_GPIO_Port, &gpio);

    /* M1 STEP + DIR */
    HAL_GPIO_WritePin(M1_STEP_GPIO_Port, M1_STEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, GPIO_PIN_RESET);
    gpio.Pin = M1_STEP_Pin;  HAL_GPIO_Init(M1_STEP_GPIO_Port, &gpio);
    gpio.Pin = M1_DIR_Pin;   HAL_GPIO_Init(M1_DIR_GPIO_Port, &gpio);

    /* M2 STEP + DIR */
    HAL_GPIO_WritePin(M2_STEP_GPIO_Port, M2_STEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, GPIO_PIN_RESET);
    gpio.Pin = M2_STEP_Pin;  HAL_GPIO_Init(M2_STEP_GPIO_Port, &gpio);
    gpio.Pin = M2_DIR_Pin;   HAL_GPIO_Init(M2_DIR_GPIO_Port, &gpio);

    /* M3 STEP + DIR */
    HAL_GPIO_WritePin(M3_STEP_GPIO_Port, M3_STEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_DIR_GPIO_Port, M3_DIR_Pin, GPIO_PIN_RESET);
    gpio.Pin = M3_STEP_Pin;  HAL_GPIO_Init(M3_STEP_GPIO_Port, &gpio);
    gpio.Pin = M3_DIR_Pin;   HAL_GPIO_Init(M3_DIR_GPIO_Port, &gpio);
}

bool Drv_Stepper_EnableAll(bool enable)
{
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

/** 输出一次 STEP 并累加驱动层计数；脉冲时序属于后续硬件验收项目。 */
bool Drv_Stepper_Step(StepperDevice_t *motor)
{
    if (!motor) return false;

    HAL_GPIO_WritePin(motor->step_port, motor->step_pin, GPIO_PIN_SET);
    __NOP(); __NOP(); __NOP(); __NOP();
    HAL_GPIO_WritePin(motor->step_port, motor->step_pin, GPIO_PIN_RESET);

    motor->absolute_position += (motor->current_dir == STEPPER_DIRECTION_CW) ? 1 : -1;
    return true;
}

/* 设备实例访问入口。 */
bool Drv_Stepper_WriteStepLevel(StepperDevice_t *motor, bool high)
{
    if (!motor) return false;
    HAL_GPIO_WritePin(motor->step_port,motor->step_pin,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}

StepperDevice_t *Drv_Stepper_GetM1(void) { return &s_motor_m1; }
StepperDevice_t *Drv_Stepper_GetM2(void) { return &s_motor_m2; }
StepperDevice_t *Drv_Stepper_GetM3(void) { return &s_motor_m3; }
