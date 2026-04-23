/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bsp_led.h"
#include "bsp_stepper.h"
#include "bsp_tmc2209.h"
#include "motor_core.h"
#include "motion_planner.h"
#include "gcode_parser.h"
#include "cmd_executor.h"
#include "bsp_uart1.h"
#include <stdio.h>
#include "bsp_ps2.h"
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
  typedef enum {
    SYS_MODE_GCODE,  // 写字机模式 (长线段，S型加减速)
    SYS_MODE_PS2     // 手柄遥控模式 (微步进，无延迟响应)
  } SystemMode_t;
  SystemMode_t current_sys_mode = SYS_MODE_GCODE;
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
  /* USER CODE BEGIN 2 */
  BSP_LED_Init(); // 初始化LED
  BSP_Stepper_Init(); // 初始化步进电机驱动，设置默认状�??
  BSP_UART1_Init(); // 初始�??? UART1 接收 G-code 指令
  BSP_UART1_SendString("System Boot Up OK!\r\n");
  BSP_PS2_Init(); // 初始�? PS2 手柄接口

   // 延时等待底层稳定
  HAL_Delay(100);
  printf("\r\n================================\r\n");
  printf("System Boot Up OK! Gcode Mode\r\n");
  printf("================================\r\n");

  BSP_Stepper_Enable(&Motor_M1, true);// 启用电机1
  BSP_Stepper_Enable(&Motor_M2, true);// 启用电机2
  BSP_Stepper_Enable(&Motor_M3, true);// 启用电机3

  extern UART_HandleTypeDef huart6; // 确保声明了你的串口句�?????
  // �?????0 (底座)：需要最大的力，16细分
  BSP_TMC2209_ConfigNode(&huart6, 0,  16, 28, 15); 

  // �?????1 (大臂)：中等力�?????16细分
  BSP_TMC2209_ConfigNode(&huart6, 1, 16, 28, 15); 

  // �?????2 (小臂)：负载极小，但为了极致顺滑，可以�????? 32 细分，小电流
  BSP_TMC2209_ConfigNode(&huart6, 2, 16, 28, 15);

  Motor_Core_Init(); //初始化环形缓冲区
  Motion_Planner_Init(0.0f, 185.0f, 240.0f); // 设置初始位置�?????(0, 185, 240)，即机械臂的默认位置
  Cmd_Executor_Init(0.0f, 185.0f, 240.0f);  // 初始化执行器
  


  extern TIM_HandleTypeDef htim6; 
  HAL_TIM_Base_Start_IT(&htim6);// 启动定时�?????6的中断，�?????始处理运动帧

  PS2_Data_t my_ps2;
  uint16_t last_buttons = 0xFFFF; // 记录上次的按键状态，用于检测"单击"动作

  char rx_line[256];
  GCodeFrame_t gcode_frame;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  LedState_t my_pattern[LED_COUNT] = {LED_OFF, LED_OFF, LED_ON, LED_OFF};
      BSP_LED_SetAllStates(my_pattern);

      // 修复 Bug 3：初始化清零，防止读取失败时使用垃圾内存
      PS2_Data_t my_ps2 = {0}; 
      
      // 记录手柄本帧是否通讯成功
      bool is_ps2_connected = BSP_PS2_ReadData(&my_ps2);

      // ========================================================
      // 模块 1：手柄读取与模式安全切换
      // ========================================================
      if (is_ps2_connected)
      {
          // 检测 SELECT 键的"单击"
          if ((last_buttons & PS2_BTN_SELECT) && !(my_ps2.buttons & PS2_BTN_SELECT))
          {
              if (Motor_Buffer_GetCount() == 0) {
                  current_sys_mode = (current_sys_mode == SYS_MODE_GCODE) ? SYS_MODE_PS2 : SYS_MODE_GCODE;
                  printf("\r\n>>> MODE SWITCHED TO: [%s] <<<\r\n", 
                         (current_sys_mode == SYS_MODE_GCODE) ? "G-CODE" : "PS2 TELEOP");
              } else {
                  printf("Warning: Please wait for motors to stop before switching mode!\r\n");
              }
          }
          last_buttons = my_ps2.buttons;
      }

      // ========================================================
      // 模块 2：状态机分流处理
      // ========================================================
      if (current_sys_mode == SYS_MODE_GCODE)
      {
          // 修复 Bug 4：去除了外面多余的 BSP_UART1_ReadLine 拦截
          if (BSP_UART1_ReadLine(rx_line, sizeof(rx_line))) 
          {
              printf(">> Received raw line: [%s]\r\n", rx_line);
              if (GCode_ParseLine(rx_line, &gcode_frame)) {
                  Cmd_Executor_Run(&gcode_frame);
                  printf("ok\r\n");
              } else {
                  printf("error: Parse failed!\r\n");
              }
          }
      }
      else if (current_sys_mode == SYS_MODE_PS2)
      {
          // 必须确保手柄正常通讯，才能解析摇杆（防止断连后乱飞）
          if (is_ps2_connected) 
          {
              float dx = 0.0f, dy = 0.0f, dz = 0.0f;

              int joy_ly = 128 - my_ps2.LY; 
              int joy_lx = my_ps2.LX - 128; 
              int joy_ry = 128 - my_ps2.RY; 

              if (abs(joy_ly) > 15) dx = (joy_ly / 128.0f) * 1.5f; 
              if (abs(joy_lx) > 15) dy = (joy_lx / 128.0f) * 1.5f; 
              if (abs(joy_ry) > 15) dz = (joy_ry / 128.0f) * 1.5f; 

              if (dx != 0.0f || dy != 0.0f || dz != 0.0f) 
              {
                  Motion_Planner_TeleopStep(dx, dy, dz);
              }
          }
          HAL_Delay(5); 
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
