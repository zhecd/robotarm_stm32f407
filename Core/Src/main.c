/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body / 主程序
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bsp/bsp_led.h"
#include "bsp/bsp_stepper.h"
#include "bsp/bsp_tmc2209.h"
#include "bsp/bsp_uart1.h"
#include "bsp/bsp_ps2.h"
#include "bsp/bsp_gripper.h"
#include "bsp/bsp_as5600.h"
#include "bsp/bsp_homing.h"

#include "control/ctrl_motion_engine.h"
#include "control/ctrl_planner.h"
#include "control/ctrl_closed_loop.h"
#include "control/ctrl_compensation.h"

#include "app/app_teleop.h"
#include "app/app_gcode_parser.h"
#include "app/app_gcode_exec.h"
#include "app/app_calibration.h"

#include "common/common.h"
#include "common/robot_config.h"

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/** 1 Hz encoder status reporting (G-code mode only). */
static void Task_EncoderReport(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tick < 1000) return;
    last_tick = now;

    float cur[CL_AXIS_COUNT];
    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        cur[i] = 0.0f;
        if (Ctrl_ClosedLoop_IsAxisEnabled(i)) {
            AS5600_Dev_t *enc = Ctrl_ClosedLoop_GetEncoder(i);
            cur[i] = (BSP_AS5600_Update(enc) == ERR_OK) ? enc->angle_deg : 0.0f;
        }
    }

    static float  last[CL_AXIS_COUNT];
    static bool   first = true;
    float diff = fabsf(cur[0] - last[0])
               + fabsf(cur[1] - last[1])
               + fabsf(cur[2] - last[2]);

    if (first || diff > 1.0f) {
        first = false;
        BSP_AS5600_PrintStatus();
        for (int i = 0; i < CL_AXIS_COUNT; i++) last[i] = cur[i];
    }
}

/** 50 Hz PID closed-loop position hold (G-code mode only). */
static void Task_ClosedLoop(void)
{
    static uint32_t last_cl = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_cl < 20) return;
    last_cl = now;
    Ctrl_ClosedLoop_Update();
}

/** G-code receive + execute pipeline (G-code mode only). */
static void Task_GCode(void)
{
    char          line[256];
    GCodeFrame_t  frame;

    if (!BSP_UART1_ReadLine(line, sizeof(line))) return;

    if (App_GCodeParser_ParseLine(line, &frame)) {
        App_GCodeExec_Run(&frame);
        Ctrl_Compensation_Execute();
        HAL_Delay(100);
        Ctrl_ClosedLoop_SyncTarget();
        printf("ok\r\n");
    } else {
        printf("error: Parse failed!\r\n");
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
         HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM6_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  /* ── BSP initialization ── */
  BSP_LED_Init();
  BSP_Stepper_Init();
  BSP_UART1_Init();
  BSP_UART1_SendString("System Boot Up OK!\r\n");
  BSP_PS2_Init();

  extern TIM_HandleTypeDef htim2;
  BSP_Gripper_Init(BSP_Gripper_GetHandle(), &htim2, TIM_CHANNEL_2);

  BSP_AS5600_Init();

  HAL_Delay(100);
  printf("\r\n================================\r\n");
  printf("System Boot Up OK! Gcode Mode\r\n");
  printf("================================\r\n");

  /* ── Motor drivers ── */
  BSP_Stepper_Enable(BSP_Stepper_GetM1(), true);
  BSP_Stepper_Enable(BSP_Stepper_GetM2(), true);
  BSP_Stepper_Enable(BSP_Stepper_GetM3(), true);

  extern UART_HandleTypeDef huart6;
  BSP_TMC2209_ConfigNode(&huart6, 0, TMC_DEFAULT_MICROSTEPS, TMC_DEFAULT_IRUN, TMC_DEFAULT_IHOLD);
  BSP_TMC2209_ConfigNode(&huart6, 1, TMC_DEFAULT_MICROSTEPS, TMC_DEFAULT_IRUN, TMC_DEFAULT_IHOLD);
  BSP_TMC2209_ConfigNode(&huart6, 2, TMC_DEFAULT_MICROSTEPS, TMC_DEFAULT_IRUN, TMC_DEFAULT_IHOLD);

  /* ── Homing ── */
  BSP_Homing_Execute();

  /* ── Control layer initialization ── */
  Ctrl_MotionEngine_Init();
  Ctrl_Planner_Init(0.0f, 185.0f, 240.0f);
  App_GCodeExec_Init(0.0f, 185.0f, 240.0f);

  /* ── Application layer ── */
  App_Teleop_Init();
  Ctrl_ClosedLoop_Init();
  App_Calibration_Execute();

  /* ── Mode indicator LED ── */
  BSP_LED_SetState(LED_0, LED_ON);
  BSP_LED_SetState(LED_1, LED_OFF);

  /* ── Start motion engine interrupt (TIM6, 50 kHz) ── */
  extern TIM_HandleTypeDef htim6;
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      SystemMode_t mode = App_Teleop_GetMode();

      if (mode == SYS_MODE_GCODE) {
          Task_EncoderReport();
          Task_ClosedLoop();
      }

      App_Teleop_Task();

      if (mode == SYS_MODE_GCODE) {
          Task_GCode();
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* PE2 key: mode switch GCode(LED0) <-> PS2 teleop(LED1) */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != KEY_MODE_Pin)
        return;

    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tick < 50)
        return;
    last_tick = now;

    if (Ctrl_MotionEngine_IsRunning()) {
        printf("Warning: Motors moving, cannot switch mode!\r\n");
        return;
    }

    App_Teleop_ToggleMode();

    SystemMode_t mode = App_Teleop_GetMode();
    BSP_LED_SetState(LED_0, (mode == SYS_MODE_GCODE) ? LED_ON : LED_OFF);
    BSP_LED_SetState(LED_1, (mode == SYS_MODE_PS2)   ? LED_ON : LED_OFF);

    printf("\r\n>>> MODE: [%s] <<<\r\n",
           (mode == SYS_MODE_GCODE) ? "G-CODE" : "PS2 TELEOP");
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
