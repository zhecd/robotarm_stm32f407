/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body / 濞戞捁宕甸埢鍏兼�?  ******************************************************************************
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
#include "bsp/bsp_uart1.h"
#include "device/dev_input.h"
#include "device/dev_joint.h"
#include "device/dev_limit_switch.h"
#include "service/svc_gripper.h"
#include "service/svc_homing.h"

#include "service/control/ctrl_motion_engine.h"
#include "service/control/ctrl_planner.h"
#include "service/control/ctrl_closed_loop.h"

#include "app/app_teleop.h"
#include "app/app_gcode_parser.h"
#include "app/app_gcode_exec.h"
#include "app/app_calibration.h"

#include "common/robot_math.h"
#include "common/robot_home_pose.h"
#include "common/robot_config.h"

#include <stdio.h>
#include <ctype.h>

/* Mode indicators: G-code uses LED0; PS2 teleoperation uses LED1. */
#define MODE_LED_GCODE LED_0
#define MODE_LED_PS2   LED_1

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

static volatile bool s_mode_switch_requested = false;
static bool s_wait_for_motion = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/** 50 Hz encoder reading for multi-turn wrap tracking / 50Hz 缂傚倹鐗滈悥婊堝闯閵婎煈鍤㈤柛娆愮墱閺併倖绂嶆惔鈽嗘▼闁革箑鐗愮粣锟犵叒?*/
static void Task_EncoderRead(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tick < 20) return;
    last_tick = now;

    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        if (Ctrl_ClosedLoop_IsAxisEnabled(i)) {
            float angle;
            Ctrl_ClosedLoop_GetAxisAngle(i, &angle);
        }
    }
}

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
            cur[i] = (Ctrl_ClosedLoop_GetAxisAngle(i, &cur[i])) ? cur[i] : 0.0f;
        }
    }

    static float  last[CL_AXIS_COUNT];
    static bool   first = true;
    float diff = fabsf(cur[0] - last[0])
               + fabsf(cur[1] - last[1])
               + fabsf(cur[2] - last[2]);

    if (first || diff > 1.0f) {
        first = false;
        Dev_Joint_PrintFeedbackStatus();
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

static void ReportM114(void)
{
    float x, y, z;
    float motor[CL_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
    float joint[CL_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
    App_GCodeExec_GetPlannedPosition(&x, &y, &z);

    for (int i = 0; i < CL_AXIS_COUNT; i++) {
        if (Ctrl_ClosedLoop_GetAxisAngle(i, &motor[i]))
            joint[i] = RobotHomePose_MotorDegToJointDeg((RobotHomeAxis_t)i, motor[i]);
    }

    printf("M114 PLAN X:%.2f Y:%.2f Z:%.2f J:%.2f,%.2f,%.2f ENC:%.2f,%.2f,%.2f\r\n",
           x, y, z, joint[0], joint[1], joint[2], motor[0], motor[1], motor[2]);
}

static void ReportM119(void)
{
    int m1 = Dev_LimitSwitch_IsTriggered(DEV_JOINT_M1);
    int m2 = Dev_LimitSwitch_IsTriggered(DEV_JOINT_M2);
    int m3 = Dev_LimitSwitch_IsTriggered(DEV_JOINT_M3);
    printf("M119 M1:%s M2:%s M3:%s\r\n",
           m1 ? "TRIGGERED" : "OPEN",
           m2 ? "TRIGGERED" : "OPEN",
           m3 ? "TRIGGERED" : "OPEN");
}

/** G-code receive + execute pipeline (G-code mode only). */
static void Task_GCode(void)
{
    char          line[256];
    GCodeFrame_t  frame;

    /* A second guard makes this task safe even if the mode changes between
       scheduler checks.  UART input is intentionally left unread in PS2 mode. */
    if (App_Teleop_GetMode() != SYS_MODE_GCODE && !Ctrl_MotionEngine_HasFault()) return;
    if (s_wait_for_motion) {
        if (Ctrl_MotionEngine_HasFault()) {
            s_wait_for_motion = false;
            printf("error: M400 aborted by safety fault\r\n");
        } else if (!Ctrl_MotionEngine_IsRunning() && Ctrl_MotionEngine_GetQueueCount() == 0U) {
            s_wait_for_motion = false;
            printf("ok\r\n");
        }
        return;
    }
    if (BSP_UART1_TakeLineTimeout())
        printf("error: incomplete command; CRLF required\r\n");
    if (BSP_UART1_TakeRxOverflow())
        printf("error: UART RX overflow; command discarded\r\n");
    if (!BSP_UART1_ReadLine(line, sizeof(line))) return;

    /* Ignore empty/control-only frames left by terminal line-ending timing.
       They are not G-code errors and must not produce a false alarm. */
    bool has_printable = false;
    for (const char *p = line; *p != '\0'; p++) {
        if (isprint((unsigned char)*p)) {
            has_printable = true;
            break;
        }
    }
    if (!has_printable) return;

    if (App_GCodeParser_ParseLine(line, &frame) && frame.type == GCMD_M114) {
        ReportM114();
        return;
    }
    if (App_GCodeParser_ParseLine(line, &frame) && frame.type == GCMD_M119) {
        ReportM119();
        return;
    }
    if (App_GCodeParser_ParseLine(line, &frame) && frame.type == GCMD_M400) {
        if (Ctrl_MotionEngine_HasFault()) {
            printf("error: M400 rejected by safety fault\r\n");
        } else if (!Ctrl_MotionEngine_IsRunning() && Ctrl_MotionEngine_GetQueueCount() == 0U) {
            printf("ok\r\n");
        } else {
            s_wait_for_motion = true;
        }
        return;
    }

    if (App_GCodeParser_ParseLine(line, &frame) && frame.type == GCMD_M999) {
        if (!Ctrl_MotionEngine_HasFault()) {
            printf("error: M999 requires a safety fault\r\n");
            return;
        }
        Ctrl_MotionEngine_EnableLimitMonitoring(false);
        if (!Svc_Homing_Execute()) {
            Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_NONE);
            printf("error: M999 homing failed\r\n");
            return;
        }
        Ctrl_MotionEngine_Init();
        if (Ctrl_Planner_Init(g_robot_home_pose.x_mm, g_robot_home_pose.y_mm, g_robot_home_pose.z_mm) != ERR_OK) {
            Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_NONE);
            printf("error: M999 planner reset failed\r\n");
            return;
        }
        App_GCodeExec_Init(g_robot_home_pose.x_mm, g_robot_home_pose.y_mm, g_robot_home_pose.z_mm);
        Ctrl_ClosedLoop_Init();
        if (!App_Calibration_Execute()) {
            Ctrl_MotionEngine_EmergencyStopWithReason(MOTION_FAULT_ENCODER);
            printf("error: M999 encoder calibration failed\r\n");
            return;
        }
        Ctrl_MotionEngine_ClearFault();
        Ctrl_MotionEngine_EnableLimitMonitoring(true);
        printf("[HomePose] XYZ=%.1f,%.1f,%.1f J=%.1f,%.1f,%.1f\r\n",
               g_robot_home_pose.x_mm, g_robot_home_pose.y_mm, g_robot_home_pose.z_mm,
               g_robot_home_pose.joint_deg[0], g_robot_home_pose.joint_deg[1],
               g_robot_home_pose.joint_deg[2]);
        printf("M999OK\r\n");
        return;
    }

    if (App_GCodeParser_ParseLine(line, &frame)) {
        ErrorCode_t status = App_GCodeExec_Run(&frame);
        if (status != ERR_OK) {
            printf("error: command rejected (%d)\r\n", (int)status);
            return;
        }
        /* Static compensation contains blocking waits and must not run from
           the command path.  The periodic closed-loop task provides position
           hold; only a Cartesian command needs a new closed-loop target. */
        if (frame.type == GCMD_G0 || frame.type == GCMD_G1)
            Ctrl_ClosedLoop_SyncTarget();

        switch (frame.type) {
        case GCMD_M3: printf("M3OK\r\n"); break;
        case GCMD_M5: printf("M5OK\r\n"); break;
        default:      printf("ok\r\n");   break;
        }
    } else {
        printf("error: Parse failed!\r\n");
    }
}

/* Execute mode changes in the foreground, not inside the GPIO interrupt.
   This keeps UART output and application state out of interrupt context. */
static void Task_ModeSwitch(void)
{
    static uint32_t last_switch_ms = 0U;
    if (!s_mode_switch_requested) return;
    s_mode_switch_requested = false;

    uint32_t now = HAL_GetTick();
    if ((now - last_switch_ms) < 200U) return;
    last_switch_ms = now;

    if (Ctrl_MotionEngine_IsRunning() || Ctrl_MotionEngine_GetQueueCount() != 0U) {
        printf("Warning: Motors moving, cannot switch mode!\r\n");
        return;
    }

    App_Teleop_ToggleMode();
    SystemMode_t mode = App_Teleop_GetMode();
    BSP_LED_SetState(MODE_LED_GCODE, (mode == SYS_MODE_GCODE) ? LED_ON : LED_OFF);
    BSP_LED_SetState(MODE_LED_PS2,   (mode == SYS_MODE_PS2)   ? LED_ON : LED_OFF);
    printf("\r\n>>> MODE: [%s] <<<\r\n",
           (mode == SYS_MODE_GCODE) ? "G-CODE" : "PS2 TELEOP");
}

static void UpdateModeIndicator(SystemMode_t mode)
{
    static SystemMode_t last_mode = (SystemMode_t)-1;
    if (mode == last_mode) return;
    last_mode = mode;
    BSP_LED_SetState(MODE_LED_GCODE, (mode == SYS_MODE_GCODE) ? LED_ON : LED_OFF);
    BSP_LED_SetState(MODE_LED_PS2,   (mode == SYS_MODE_PS2)   ? LED_ON : LED_OFF);
}

static const char *MotionFaultText(MotionFaultReason_t reason)
{
    switch (reason) {
    case MOTION_FAULT_LIMIT_SWITCH: return "limit switch";
    case MOTION_FAULT_ENCODER:      return "encoder communication";
    case MOTION_FAULT_SOFT_LIMIT:   return "actual joint soft limit";
    case MOTION_FAULT_QUEUE_TIMEOUT:return "planner queue timeout";
    default:                        return "unspecified";
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

  /* 闁冲厜鍋撻柍鍏夊�?BSP initialization 闁冲厜鍋撻柍鍏夊�?*/
  BSP_LED_Init();
  Dev_Joint_Init();
  BSP_UART1_Init();
  BSP_UART1_SendString("System Boot Up OK!\r\n");
  Dev_Input_Init();

  extern TIM_HandleTypeDef htim2;
  Svc_Gripper_Init(&htim2);

  HAL_Delay(100);
  printf("\r\n================================\r\n");
  printf("System Boot Up OK! Gcode Mode\r\n");
  printf("================================\r\n");

  /* 闁冲厜鍋撻柍鍏夊�?Motor drivers 闁冲厜鍋撻柍鍏夊�?*/
  Dev_Joint_EnableAll(true);

  extern UART_HandleTypeDef huart6;
  Dev_Joint_ConfigureDrivers(&huart6);
  /* 闁冲厜鍋撻柍鍏夊�?Homing 闁冲厜鍋撻柍鍏夊�?*/
  if (!Svc_Homing_Execute()) {
      printf("error: homing failed; motion disabled\r\n");
      Dev_Joint_EnableAll(false);
      (void)BSP_UART1_FlushTx(100U);
      Error_Handler();
  }

  /* 闁冲厜鍋撻柍鍏夊�?Control layer initialization 闁冲厜鍋撻柍鍏夊�?*/
  Ctrl_MotionEngine_Init();
  if (Ctrl_Planner_Init(g_robot_home_pose.x_mm, g_robot_home_pose.y_mm, g_robot_home_pose.z_mm) != ERR_OK) {
      printf("error: initial pose is unreachable; motion disabled\r\n");
      Dev_Joint_EnableAll(false);
      (void)BSP_UART1_FlushTx(100U);
      Error_Handler();
  }
  App_GCodeExec_Init(g_robot_home_pose.x_mm, g_robot_home_pose.y_mm, g_robot_home_pose.z_mm);

  /* 闁冲厜鍋撻柍鍏夊�?Application layer 闁冲厜鍋撻柍鍏夊�?*/
  App_Teleop_Init();
  Ctrl_ClosedLoop_Init();
  if (!App_Calibration_Execute()) {
      printf("error: encoder calibration failed; motion disabled\r\n");
      Dev_Joint_EnableAll(false);
      (void)BSP_UART1_FlushTx(100U);
      Error_Handler();
  }
  Ctrl_MotionEngine_ClearFault();
  Ctrl_MotionEngine_EnableLimitMonitoring(true);
  printf("[HomePose] XYZ=%.1f,%.1f,%.1f J=%.1f,%.1f,%.1f\r\n",
         g_robot_home_pose.x_mm, g_robot_home_pose.y_mm, g_robot_home_pose.z_mm,
         g_robot_home_pose.joint_deg[0], g_robot_home_pose.joint_deg[1],
         g_robot_home_pose.joint_deg[2]);

  /* 闁冲厜鍋撻柍鍏夊�?Mode indicator LED 闁冲厜鍋撻柍鍏夊�?*/
  BSP_LED_SetState(MODE_LED_GCODE, LED_ON);
  BSP_LED_SetState(MODE_LED_PS2, LED_OFF);

  /* 闁冲厜鍋撻柍鍏夊�?Start motion engine interrupt (TIM6, 50 kHz) 闁冲厜鍋撻柍鍏夊�?*/
  extern TIM_HandleTypeDef htim6;
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      static bool fault_reported = false;
      Ctrl_MotionEngine_ServiceSafety();
      if (BSP_UART1_TakeTxOverflow())
          BSP_UART1_SendString("warning: UART TX queue overflow; log dropped\r\n");
      if (Ctrl_MotionEngine_HasFault()) {
          BSP_LED_SetState(MODE_LED_GCODE, LED_OFF);
          BSP_LED_SetState(MODE_LED_PS2, LED_ON);
          if (!fault_reported) {
              printf("error: motion safety stop (%s); re-home required\r\n",
                     MotionFaultText(Ctrl_MotionEngine_GetFaultReason()));
              fault_reported = true;
          }
          /* Keep communications and the gripper available.  Cartesian motion
             remains rejected by the faulted motion engine. */
          Task_ModeSwitch();
          Svc_Gripper_IdleStop();
          Task_GCode();
          continue;
      }
      fault_reported = false;
      Task_ModeSwitch();
      App_Teleop_Task();
      SystemMode_t mode = App_Teleop_GetMode();
      UpdateModeIndicator(mode);

      /* G-code is disabled in PS2 mode.  Discard bytes received in this
         mode so a stale command cannot run after switching back. */
      if (mode == SYS_MODE_PS2)
          BSP_UART1_DiscardRx();

      if (mode == SYS_MODE_GCODE) {
          Task_EncoderReport();
          Task_ClosedLoop();
      } else {
          /* In G-code mode the closed-loop task already samples the encoders.
             Poll only in teleop mode to avoid redundant I2C transactions. */
          Task_EncoderRead();
      }

      Svc_Gripper_IdleStop();

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
    if (GPIO_Pin == M1_STOP_Pin || GPIO_Pin == M2_STOP_Pin || GPIO_Pin == M3_STOP_Pin) {
        Ctrl_MotionEngine_NotifyLimitSwitch(GPIO_Pin);
        return;
    }

    if (GPIO_Pin == KEY_MODE_Pin)
        s_mode_switch_requested = true;
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
