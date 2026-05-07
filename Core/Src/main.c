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
#include "i2c.h"
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
#include <math.h>
#include "bsp_ps2.h"
#include <stdlib.h>
#include "app_teleop.h"
#include "bsp_gripper.h"
#include "bsp_as5600.h"
#include "closed_loop.h"

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

void App_Align_Coordinates(void);
void App_Static_Compensation(void);

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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  BSP_LED_Init(); // 初始化LED
  BSP_Stepper_Init(); // 初始化步进电机驱动，设置默认状�??
  BSP_UART1_Init(); // 初始�?????? UART1 接收 G-code 指令
  BSP_UART1_SendString("System Boot Up OK!\r\n");
  BSP_PS2_Init(); // 初始�??? PS2 手柄接口
  extern TIM_HandleTypeDef htim2; 
  BSP_Gripper_Init(&hgripper, &htim2, TIM_CHANNEL_2); // 绑定 PA1 (TIM2_CH2)
  BSP_AS5600_Init(); // 初始化三个AS5600编码器，记录上电零点

   // 延时等待底层稳定
  HAL_Delay(100);
  printf("\r\n================================\r\n");
  printf("System Boot Up OK! Gcode Mode\r\n");
  printf("================================\r\n");

  BSP_Stepper_Enable(&Motor_M1, true);// 启用电机1
  BSP_Stepper_Enable(&Motor_M2, true);// 启用电机2
  BSP_Stepper_Enable(&Motor_M3, true);// 启用电机3

  extern UART_HandleTypeDef huart6; // 确保声明了你的串口句�????????
  // �????????0 (底座)：需要最大的力，16细分
  BSP_TMC2209_ConfigNode(&huart6, 0,  16, 28, 15); 

  // �????????1 (大臂)：中等力�????????16细分
  BSP_TMC2209_ConfigNode(&huart6, 1, 16, 28, 15); 

  // �????????2 (小臂)：负载极小，但为了极致顺滑，可以�???????? 32 细分，小电流
  BSP_TMC2209_ConfigNode(&huart6, 2, 16, 28, 15);

  Motor_Core_Init(); //初始化环形缓冲区
  Motion_Planner_Init(0.0f, 185.0f, 240.0f); // 设置初始位置�????????(0, 185, 240)，即机械臂的默认位置
  Cmd_Executor_Init(0.0f, 185.0f, 240.0f);  // 初始化执行器

  App_Teleop_Init();

  App_Align_Coordinates(); // 传感器坐标系与理论步数坐标对�?
  CL_Init();               // 初始化三�? PID 闭环控制�?


  extern TIM_HandleTypeDef htim6; 
  HAL_TIM_Base_Start_IT(&htim6);// 启动定时�???????6的中断，�???????始处理运动帧

  char rx_line[256];
  GCodeFrame_t gcode_frame;

  LedState_t my_pattern[LED_COUNT] = {LED_OFF, LED_OFF, LED_ON, LED_OFF};
  BSP_LED_SetAllStates(my_pattern);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* 1Hz 编码器数据上�? */
      {
          static uint32_t last_tick = 0;
          uint32_t now = HAL_GetTick();
          if (now - last_tick >= 1000) {
              last_tick = now;
              BSP_AS5600_Update(&Encoder_M1);
              BSP_AS5600_Update(&Encoder_M2);
              BSP_AS5600_Update(&Encoder_M3);
              /* 用整数避�? newlib-nano 不支�? %%f; 手动处理符号 */
              {
                  int d1 = (int)(Encoder_M1.angle_deg * 10.0f);
                  int d2 = (int)(Encoder_M2.angle_deg * 10.0f);
                  int d3 = (int)(Encoder_M3.angle_deg * 10.0f);
                  printf("ENC M1:%s%d.%d M2:%s%d.%d M3:%s%d.%d\r\n",
                         d1 < 0 ? "-" : "", abs(d1) / 10, abs(d1) % 10,
                         d2 < 0 ? "-" : "", abs(d2) / 10, abs(d2) % 10,
                         d3 < 0 ? "-" : "", abs(d3) / 10, abs(d3) % 10);
              }
          }
      }

      /* 50Hz PID 闭环位置保持 */
      {
          static uint32_t last_cl = 0;
          uint32_t now = HAL_GetTick();
          if (now - last_cl >= 20) {
              last_cl = now;
              CL_Update();
          }
      }

      App_Teleop_Task();
      // 2. 运行 G代码 接收任务
      // (未来这部分也可以封装�??? App_Gcode_Task())
      if (current_sys_mode == SYS_MODE_GCODE)
      {
          if (BSP_UART1_ReadLine(rx_line, sizeof(rx_line))) 
          {
              if (GCode_ParseLine(rx_line, &gcode_frame)) {
                  Cmd_Executor_Run(&gcode_frame);
                  App_Static_Compensation();
                  CL_SetTargetsFromTheory();
                  printf("ok\r\n");
              } else {
                  printf("error: Parse failed!\r\n");
              }
          }
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

/* 坐标对齐：将传感器物理零位与理论步数零位同步 */
void App_Align_Coordinates(void)
{
    Motor_Core_ResetTheorySteps();

    bool ok1 = BSP_AS5600_SetZero(&Encoder_M1);
    bool ok2 = BSP_AS5600_SetZero(&Encoder_M2);
    bool ok3 = BSP_AS5600_SetZero(&Encoder_M3);

    printf("[Encoder] M1(I2C1):%s M2(I2C2):%s M3(I2C3):%s\r\n",
           ok1 ? "OK" : "FAIL",
           ok2 ? "OK" : "FAIL",
           ok3 ? "OK" : "FAIL");
}

/*
 * 静�?�位置误差补偿：持续迭代直到编码器收敛到理论目标位置�?
 *
 * 架构保证（关键）�?
 *   g_theory_steps 只由 MotionPlanner Push 帧时累加，补偿帧直接 Push 不经�? Planner�?
 *   因此 theory 天然只代�?"规划层命令�??"，不会被补偿运动污染�?
 *   补偿循环中取�?�? theory 快照作为固定目标，只追踪编码器是否到达该目标�?
 */
#define COMP_DEADBAND_DEG     1.0f    /* 死区阈�?? (�?) */
#define COMP_SPEED_DIV        50      /* 补偿速度: TIM6=50kHz, DIV=50 �? 1000�?/�? */
#define COMP_MIN_TICKS        100U    /* �?�? tick �? (2ms) */
#define COMP_WATCHDOG_ROUNDS  30      /* 安全看门�? */

void App_Static_Compensation(void)
{
    /* 等待规划层运动全部完�? */
    while (Motor_Core_IsRunning() || Motor_Buffer_GetCount() > 0) {}
    HAL_Delay(50);

    /* 快照：取�?次理论步数作为本轮补偿的固定目标（后续不再重读） */
    int32_t target_m1, target_m2, target_m3;
    Motor_Core_GetTheorySteps(&target_m1, &target_m2, &target_m3);
    /* 理论(微步) �? 电机轴角�?: ×DEGREES_PER_STEP (0.1125°/�?) */
    float target_deg_m1 = (float)target_m1 * DEGREES_PER_STEP;
    float target_deg_m2 = (float)target_m2 * DEGREES_PER_STEP;
    float target_deg_m3 = (float)target_m3 * DEGREES_PER_STEP;

    /* 诊断：两侧都换算为微步显示，避免 %%f 浮点打印不工�? */
    BSP_AS5600_Update(&Encoder_M1);
    BSP_AS5600_Update(&Encoder_M2);
    BSP_AS5600_Update(&Encoder_M3);
    printf("[CompDebug] M1:th=%ldst enc~%ldst | M2:th=%ldst enc~%ldst | M3:th=%ldst enc~%ldst\r\n",
           (long)target_m1, (long)roundf(Encoder_M1.angle_deg * STEPS_PER_DEGREE),
           (long)target_m2, (long)roundf(Encoder_M2.angle_deg * STEPS_PER_DEGREE),
           (long)target_m3, (long)roundf(Encoder_M3.angle_deg * STEPS_PER_DEGREE));

    /* 持久化编码器卡死标记：仅�? theory 变化（新规划器运动）时复�? */
    static int32_t s_last_theory_m1 = -1, s_last_theory_m2 = -1, s_last_theory_m3 = -1;
    static bool s_stuck_m1 = false, s_stuck_m2 = false, s_stuck_m3 = false;
    if (target_m1 != s_last_theory_m1 || target_m2 != s_last_theory_m2 || target_m3 != s_last_theory_m3) {
        s_stuck_m1 = s_stuck_m2 = s_stuck_m3 = false;  /* 新运�? �? 复位卡死标记 */
        s_last_theory_m1 = target_m1; s_last_theory_m2 = target_m2; s_last_theory_m3 = target_m3;
    }
    bool skip_m1 = s_stuck_m1, skip_m2 = s_stuck_m2, skip_m3 = s_stuck_m3;

    float prev_err_abs_m1 = 1e9f, prev_err_abs_m2 = 1e9f, prev_err_abs_m3 = 1e9f;

    for (int iter = 0; ; iter++)
    {
        if (!skip_m1) BSP_AS5600_Update(&Encoder_M1);
        if (!skip_m2) BSP_AS5600_Update(&Encoder_M2);
        if (!skip_m3) BSP_AS5600_Update(&Encoder_M3);

        float err_m1 = target_deg_m1 - Encoder_M1.angle_deg;
        float err_m2 = target_deg_m2 - Encoder_M2.angle_deg;
        float err_m3 = target_deg_m3 - Encoder_M3.angle_deg;

        while (err_m1 >  180.0f) err_m1 -= 360.0f;
        while (err_m1 < -180.0f) err_m1 += 360.0f;
        while (err_m2 >  180.0f) err_m2 -= 360.0f;
        while (err_m2 < -180.0f) err_m2 += 360.0f;
        while (err_m3 >  180.0f) err_m3 -= 360.0f;
        while (err_m3 < -180.0f) err_m3 += 360.0f;

        float abs_err1 = fabsf(err_m1), abs_err2 = fabsf(err_m2), abs_err3 = fabsf(err_m3);

        /* 死区�?查（跳过已标记为卡死的轴�? */
        bool m1_ok = skip_m1 || (abs_err1 <= COMP_DEADBAND_DEG);
        bool m2_ok = skip_m2 || (abs_err2 <= COMP_DEADBAND_DEG);
        bool m3_ok = skip_m3 || (abs_err3 <= COMP_DEADBAND_DEG);
        if (m1_ok && m2_ok && m3_ok) return;

        /* 逐轴�?测编码器是否无响应（误差未缩小） */
        if (!skip_m1 && iter > 0 && abs_err1 >= prev_err_abs_m1) {
            skip_m1 = s_stuck_m1 = true;
            printf("[Compensation] M1 编码器无响应, 跳过该轴\r\n");
        }
        if (!skip_m2 && iter > 0 && abs_err2 >= prev_err_abs_m2) {
            skip_m2 = s_stuck_m2 = true;
            printf("[Compensation] M2 编码器无响应, 跳过该轴\r\n");
        }
        if (!skip_m3 && iter > 0 && abs_err3 >= prev_err_abs_m3) {
            skip_m3 = s_stuck_m3 = true;
            printf("[Compensation] M3 编码器无响应, 跳过该轴\r\n");
        }
        /* 如果�?有非零目标轴都被跳过，�??�? */
        if ((target_m1 == 0 || skip_m1) && (target_m2 == 0 || skip_m2) && (target_m3 == 0 || skip_m3)) {
            printf("[Compensation] 无可补偿�?, �?出\r\n");
            return;
        }

        prev_err_abs_m1 = abs_err1;
        prev_err_abs_m2 = abs_err2;
        prev_err_abs_m3 = abs_err3;

        /* 全局发散�?测（仅对未被跳过的轴�? */
        float err_sum = (skip_m1 ? 0.0f : abs_err1) +
                        (skip_m2 ? 0.0f : abs_err2) +
                        (skip_m3 ? 0.0f : abs_err3);
        if (iter > 0 && err_sum == 0.0f) return;
        if (iter >= COMP_WATCHDOG_ROUNDS) {
            printf("[Compensation] 停止:%d轮未收敛 err~%ldst\r\n",
                   COMP_WATCHDOG_ROUNDS, (long)roundf(err_sum * STEPS_PER_DEGREE));
            return;
        }

        int32_t comp_m1 = 0, comp_m2 = 0, comp_m3 = 0;
        if (!skip_m1 && abs_err1 > COMP_DEADBAND_DEG)
            comp_m1 = (int32_t)roundf(err_m1 * STEPS_PER_DEGREE);
        if (!skip_m2 && abs_err2 > COMP_DEADBAND_DEG)
            comp_m2 = (int32_t)roundf(err_m2 * STEPS_PER_DEGREE);
        if (!skip_m3 && abs_err3 > COMP_DEADBAND_DEG)
            comp_m3 = (int32_t)roundf(err_m3 * STEPS_PER_DEGREE);

        /* 无有效补偿量则跳过推�? */
        if (comp_m1 != 0 || comp_m2 != 0 || comp_m3 != 0) {
            MotionFrame_t comp_frame;
            comp_frame.delta_m1 = comp_m1;
            comp_frame.delta_m2 = comp_m2;
            comp_frame.delta_m3 = comp_m3;

            uint32_t max_delta = (uint32_t)(abs(comp_m1));
            if ((uint32_t)abs(comp_m2) > max_delta) max_delta = (uint32_t)abs(comp_m2);
            if ((uint32_t)abs(comp_m3) > max_delta) max_delta = (uint32_t)abs(comp_m3);
            comp_frame.total_ticks = (max_delta > 0U)
                ? (max_delta * COMP_SPEED_DIV) : COMP_MIN_TICKS;
            if (comp_frame.total_ticks < COMP_MIN_TICKS)
                comp_frame.total_ticks = COMP_MIN_TICKS;

            Motor_Buffer_Push(&comp_frame);
            while (Motor_Core_IsRunning() || Motor_Buffer_GetCount() > 0) {}
            HAL_Delay(30);  /* 补偿后短暂消�? */
        }

        printf("[Compensation] #%d M1:%+ld M2:%+ld M3:%+ld "
               "(errStp:%+ld %+ld %+ld)%s%s%s\r\n",
               iter + 1,
               (long)comp_m1, (long)comp_m2, (long)comp_m3,
               (long)roundf(err_m1 * STEPS_PER_DEGREE),
               (long)roundf(err_m2 * STEPS_PER_DEGREE),
               (long)roundf(err_m3 * STEPS_PER_DEGREE),
               skip_m1 ? " [M1跳过]" : "",
               skip_m2 ? " [M2跳过]" : "",
               skip_m3 ? " [M3跳过]" : "");
    }
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
