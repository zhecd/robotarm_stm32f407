/**
 * @file    robot_config.h
 * @brief   Centralized configuration for the robotic arm controller / 机械臂控制器集中配置文件
 * @ingroup common
 *
 * Every tunable constant lives here with an #ifndef guard so it can be
 * overridden at build time via -D flags.
 * 所有可调参数均在此定义，带 #ifndef 保护，可通过 -D 编译选项覆写。
 */

#ifndef __ROBOT_CONFIG_H__
#define __ROBOT_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Motor Parameters / 电机参数
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifndef STEPS_PER_REV
#define STEPS_PER_REV           3200.0f      /* 200 steps x 16 microsteps (motor shaft) / 200步×16细分 (电机轴) */
#endif

#ifndef GEAR_RATIO
#define GEAR_RATIO              (90.0f / 20.0f)   /* 20T motor -> 90T output, 4.5:1 reduction / 20齿电机→90齿输出, 4.5:1 减速 */
#endif

#ifndef STEPS_PER_DEGREE
#define STEPS_PER_DEGREE        (STEPS_PER_REV / 360.0f)               /* Motor steps per motor-shaft degree / 每电机轴度的步数 (encoder frame) */
#endif

#ifndef DEGREES_PER_STEP
#define DEGREES_PER_STEP        (360.0f / STEPS_PER_REV)               /* Motor-shaft degrees per motor step / 每步电机轴度数 (encoder frame) */
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Kinematics (SCARA-like parallel-gripper arm) / 运动学 (SCARA 平行夹爪臂)
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifndef LINK_1_LEN
#define LINK_1_LEN              140.0f       /* Link 1 length (mm) / 连杆1长度 */
#endif

#ifndef LINK_2_LEN
#define LINK_2_LEN              140.0f       /* Link 2 length (mm) / 连杆2长度 */
#endif

#ifndef TOOL_OFFSET_R
#define TOOL_OFFSET_R           45.0f        /* Gripper horizontal offset (mm) / 夹爪水平偏移 */
#endif

#ifndef TOOL_OFFSET_Z
#define TOOL_OFFSET_Z           -40.0f       /* Gripper vertical offset (mm) / 夹爪垂直偏移 */
#endif

#ifndef BASE_HEIGHT
#define BASE_HEIGHT             140.0f       /* Desktop-to-shoulder height (mm) / 桌面到肩关节高度 */
#endif

#ifndef UNITS_PER_DEGREE
#define UNITS_PER_DEGREE       (STEPS_PER_REV * GEAR_RATIO / 360.0f)  /* Motor steps per joint degree (kinematics frame) / 每关节度的电机步数 (运动学量纲) */
#endif

#ifndef OFFSET_ROT
#define OFFSET_ROT              0.0f         /* Rotation joint offset (deg) / 旋转关节偏置 */
#endif

#ifndef OFFSET_LOW
#define OFFSET_LOW              0.0f         /* Shoulder joint offset (deg) / 肩关节偏置 */
#endif

#ifndef OFFSET_HIGH
#define OFFSET_HIGH             0.0f         /* Elbow joint offset (deg) / 肘关节偏置 */
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Motion Planner / 轨迹规划器
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifndef TICKS_PER_MS
#define TICKS_PER_MS            50U          /* TIM6 ticks per ms (50 kHz) / 每毫秒定时器节拍数 */
#endif

#ifndef LINEAR_SEGMENT_MM
#define LINEAR_SEGMENT_MM       0.25f        /* Max segment length (mm) / 最大分段长度 */
#endif

#ifndef PLANNER_SEGMENT_MS
#define PLANNER_SEGMENT_MS      4U           /* Target segment duration (ms) / 目标分段时长 */
#endif

#ifndef MIN_FRAME_TICKS
#define MIN_FRAME_TICKS         1U           /* Minimum ticks per frame / 每帧最小节拍数 */
#endif

#ifndef FRAME_STEP_MARGIN
#define FRAME_STEP_MARGIN       3U           /* Step margin to avoid overrun / 步数裕量防溢出 */
#endif

#ifndef STOP_TAIL_EXTRA_TICKS
#define STOP_TAIL_EXTRA_TICKS   2U           /* Extra ticks for end deceleration / 末端减速额外节拍 */
#endif

#ifndef END_SLOW_SEGMENTS
#define END_SLOW_SEGMENTS       2U           /* Number of deceleration segments / 减速段数量 */
#endif

#ifndef MOTOR_STEPS_PER_SEG
#define MOTOR_STEPS_PER_SEG     10U          /* Max motor steps per segment / 每段最大电机步数 */
#endif

#ifndef TELEOP_FRAME_MS
#define TELEOP_FRAME_MS         5U           /* Teleop frame duration (ms) / 遥控帧时长 */
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * PID Closed-Loop Control / PID 闭环控制
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifndef CL_KP
#define CL_KP                   0.25f        /* Proportional gain / 比例增益 */
#endif

#ifndef CL_KI
#define CL_KI                   0.05f        /* Integral gain / 积分增益 */
#endif

#ifndef CL_KD
#define CL_KD                   0.08f        /* Derivative gain / 微分增益 */
#endif

#ifndef CL_INTEGRAL_MAX
#define CL_INTEGRAL_MAX         5.0f         /* Integral windup limit / 积分饱和上限 */
#endif

#ifndef CL_DEADBAND_DEG
#define CL_DEADBAND_DEG         2.0f         /* Deadband (deg) / 死区 */
#endif

#ifndef CL_UPDATE_MS
#define CL_UPDATE_MS            20U          /* PID update period (ms) / PID 更新周期 */
#endif

#ifndef CL_I_SEP_ERR
#define CL_I_SEP_ERR            3.0f         /* Integral separation threshold (deg) / 积分分离阈值 */
#endif

#ifndef CL_MIN_TICKS
#define CL_MIN_TICKS            100U         /* Minimum correction ticks / 最小修正节拍 */
#endif

#ifndef CL_SPEED_DIV
#define CL_SPEED_DIV            200U         /* Correction speed divisor / 修正速度除数 */
#endif

#ifndef CL_EMA_ALPHA
#define CL_EMA_ALPHA            0.2f         /* EMA filter coefficient / 指数移动平均滤波系数 */
#endif

/* Conservative correction: 2-8 deg small error / 保守修正: 2-8° 小误差 */
#ifndef CL_OUTPUT_MAX_LO
#define CL_OUTPUT_MAX_LO        1.0f         /* Max output for small error (deg) / 小误差最大输出 */
#endif

#ifndef CL_COOLDOWN_LO_MS
#define CL_COOLDOWN_LO_MS       800U         /* Cooldown for small error (ms) / 小误差冷却时间 */
#endif

/* Aggressive recovery: >8 deg large error / 强力恢复: >8° 大误差 */
#ifndef CL_LARGE_ERR_DEG
#define CL_LARGE_ERR_DEG        8.0f         /* Large error threshold (deg) / 大误差阈值 */
#endif

#ifndef CL_OUTPUT_MAX_HI
#define CL_OUTPUT_MAX_HI        3.0f         /* Max output for large error (deg) / 大误差最大输出 */
#endif

#ifndef CL_COOLDOWN_HI_MS
#define CL_COOLDOWN_HI_MS       100U         /* Cooldown for large error (ms) / 大误差冷却时间 */
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Static Compensation / 静态补偿
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifndef COMP_DEADBAND_DEG
#define COMP_DEADBAND_DEG       1.0f         /* Compensation deadband (deg) / 补偿死区 */
#endif

#ifndef COMP_SPEED_DIV
#define COMP_SPEED_DIV          50           /* Compensation speed divisor / 补偿速度除数 */
#endif

#ifndef COMP_MIN_TICKS
#define COMP_MIN_TICKS          100U         /* Min compensation ticks / 最小补偿节拍 */
#endif

#ifndef COMP_WATCHDOG_ROUNDS
#define COMP_WATCHDOG_ROUNDS    30           /* Max compensation iterations / 最大补偿迭代次数 */
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Homing / 回零
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifndef HOMING_STEP_DELAY_US
#define HOMING_STEP_DELAY_US    200          /* Step pulse delay (us) / 步进脉冲延时 */
#endif

#ifndef HOMING_MAX_STEPS
#define HOMING_MAX_STEPS        8000         /* Max steps before timeout / 超时前最大步数 */
#endif

#ifndef HOMING_BACKOFF_M1_DEG
#define HOMING_BACKOFF_M1_DEG   100.0f       /* M1 backoff angle (deg) — joint / M1 回退角度 — 关节 */
#endif

#ifndef HOMING_BACKOFF_M2_DEG
#define HOMING_BACKOFF_M2_DEG   25.0f        /* M2 backoff angle (deg) — joint / M2 回退角度 — 关节 */
#endif

#ifndef HOMING_BACKOFF_M3_DEG
#define HOMING_BACKOFF_M3_DEG   10.0f        /* M3 backoff angle (deg) — joint / M3 回退角度 — 关节 */
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * G-code Executor / G-code 执行器
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifndef GCODE_DEFAULT_FEEDRATE
#define GCODE_DEFAULT_FEEDRATE  3000.0f      /* Default feedrate (mm/min) / 默认进给速度 */
#endif

#ifndef GCODE_MIN_MOVE_MM
#define GCODE_MIN_MOVE_MM       0.001f       /* Minimum move distance (mm) / 最小移动距离 */
#endif

#ifndef GCODE_MIN_DURATION_MS
#define GCODE_MIN_DURATION_MS   1U           /* Minimum move duration (ms) / 最小移动时长 */
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * PS2 Teleop / PS2 遥控
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifndef PS2_POLL_INTERVAL_MS
#define PS2_POLL_INTERVAL_MS    5U           /* Polling interval (ms) / 轮询间隔 */
#endif

#ifndef JOYSTICK_DEADZONE
#define JOYSTICK_DEADZONE       12           /* Joystick deadzone / 摇杆死区 */
#endif

#ifndef TELEOP_MAX_STEP_MM
#define TELEOP_MAX_STEP_MM      0.45f        /* Max step per teleop frame (mm) / 每遥控帧最大步长 */
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Gripper Servo / 夹爪舵机
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifndef GRIPPER_SERVO_MIN_US
#define GRIPPER_SERVO_MIN_US    500.0f       /* Min pulse width (us) / 最小脉宽 */
#endif

#ifndef GRIPPER_SERVO_MAX_US
#define GRIPPER_SERVO_MAX_US    2500.0f      /* Max pulse width (us) / 最大脉宽 */
#endif

#ifndef GRIPPER_ANGLE_OPEN
#define GRIPPER_ANGLE_OPEN      72.0f        /* Close angle (deg) / 合爪角度 */
#endif

#ifndef GRIPPER_ANGLE_CLOSE
#define GRIPPER_ANGLE_CLOSE     100.0f       /* Open angle (deg) / 张开角度 */
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * TMC2209 Driver / TMC2209 驱动
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifndef TMC_IRUN_HIGH
#define TMC_IRUN_HIGH           24U          /* High run current / 高运行电流 */
#endif

#ifndef TMC_IRUN_MED
#define TMC_IRUN_MED            16U          /* Medium run current / 中运行电流 */
#endif

#ifndef TMC_IRUN_LOW
#define TMC_IRUN_LOW            8U           /* Low run current / 低运行电流 */
#endif

#ifndef TMC_IHOLD_STRONG
#define TMC_IHOLD_STRONG        12U          /* Strong hold current / 强保持电流 */
#endif

#ifndef TMC_IHOLD_COOL
#define TMC_IHOLD_COOL          4U           /* Low hold current / 低保持电流 */
#endif

#ifndef TMC_DEFAULT_MICROSTEPS
#define TMC_DEFAULT_MICROSTEPS  16U          /* Default microstepping / 默认细分 */
#endif

#ifndef TMC_DEFAULT_IRUN
#define TMC_DEFAULT_IRUN        28U          /* Default run current / 默认运行电流 */
#endif

#ifndef TMC_DEFAULT_IHOLD
#define TMC_DEFAULT_IHOLD       15U          /* Default hold current / 默认保持电流 */
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * System / Timing / 系统与定时
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifndef RING_BUFFER_SIZE
#define RING_BUFFER_SIZE        64           /* Motion frame ring buffer size / 运动帧环形缓冲区大小 */
#endif

#ifndef UART1_RX_BUF_SIZE
#define UART1_RX_BUF_SIZE       256U         /* UART1 RX buffer size (bytes) / 串口接收缓冲区大小 */
#endif

#ifndef UART1_LINE_TIMEOUT_MS
#define UART1_LINE_TIMEOUT_MS   50U          /* Line read timeout (ms) / 行读取超时 */
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_CONFIG_H__ */
