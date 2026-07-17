/**
 * @file    robot_config.h
 * @brief   Centralized configuration for the robotic arm controller / 鏈烘鑷傛帶鍒跺櫒闆嗕腑閰嶇疆鏂囦欢
 * @ingroup common
 *
 * Every tunable constant lives here with an #ifndef guard so it can be
 * overridden at build time via -D flags.
 * 鎵€鏈夊彲璋冨弬鏁板潎鍦ㄦ瀹氫箟锛屽甫 #ifndef 淇濇姢锛屽彲閫氳�?-D 缂栬瘧閫夐」瑕嗗啓�?
 */

#ifndef __ROBOT_CONFIG_H__
#define __ROBOT_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
 * Motor Parameters / 鐢垫満鍙傛�?
 * 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?*/

#ifndef STEPS_PER_REV
#define STEPS_PER_REV           3200.0f      /* 200 steps x 16 microsteps (motor shaft) / 200姝ッ?6缁嗗�?(鐢垫満�? */
#endif

#ifndef GEAR_RATIO
#define GEAR_RATIO              (90.0f / 20.0f)   /* 20T motor -> 90T output, 4.5:1 reduction / 20榻跨數鏈衡啋90榻胯緭鍑? 4.5:1 鍑忛�?*/
#endif

#ifndef STEPS_PER_DEGREE
#define STEPS_PER_DEGREE        (STEPS_PER_REV / 360.0f)               /* Motor steps per motor-shaft degree / 姣忕數鏈鸿酱搴︾殑姝ユ暟 (encoder frame) */
#endif

#ifndef DEGREES_PER_STEP
#define DEGREES_PER_STEP        (360.0f / STEPS_PER_REV)               /* Motor-shaft degrees per motor step / 姣忔鐢垫満杞村害鏁?(encoder frame) */
#endif

/* 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
 * Kinematics (SCARA-like parallel-gripper arm) / 杩愬姩�?(SCARA 骞宠澶圭埅�?
 * 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?*/

#ifndef LINK_1_LEN
#define LINK_1_LEN              140.0f       /* Link 1 length (mm) / 杩炴�?闀垮害 */
#endif

#ifndef LINK_2_LEN
#define LINK_2_LEN              140.0f       /* Link 2 length (mm) / 杩炴�?闀垮害 */
#endif

#ifndef TOOL_OFFSET_R
#define TOOL_OFFSET_R           45.0f        /* Gripper horizontal offset (mm) / 澶圭埅姘村钩鍋忕Щ */
#endif

#ifndef TOOL_OFFSET_Z
#define TOOL_OFFSET_Z           -40.0f       /* Gripper vertical offset (mm) / 澶圭埅鍨傜洿鍋忕Щ */
#endif

#ifndef BASE_HEIGHT
#define BASE_HEIGHT             140.0f       /* Desktop-to-shoulder height (mm) / 妗岄潰鍒拌偐鍏宠妭楂樺害 */
#endif

#ifndef UNITS_PER_DEGREE
#define UNITS_PER_DEGREE       (STEPS_PER_REV * GEAR_RATIO / 360.0f)  /* Motor steps per joint degree (kinematics frame) / 姣忓叧鑺傚害鐨勭數鏈烘�?(杩愬姩瀛﹂噺绾? */
#endif

#ifndef OFFSET_ROT
#define OFFSET_ROT              0.0f         /* Rotation joint offset (deg) / 鏃嬭浆鍏宠妭鍋忕�?*/
#endif

#ifndef OFFSET_LOW
#define OFFSET_LOW              0.0f         /* Shoulder joint offset (deg) / 鑲╁叧鑺傚亸�?*/
#endif

#ifndef OFFSET_HIGH
#define OFFSET_HIGH             0.0f         /* Elbow joint offset (deg) / 鑲樺叧鑺傚亸�?*/
#endif

/* Software joint limits.  Fill these values from measured safe joint ranges
 * after homing; do not enable this feature with guessed mechanical limits. */
#ifndef JOINT_LIMITS_ENABLED
#define JOINT_LIMITS_ENABLED     1U
#endif

#ifndef ROT_MIN_DEG
#define ROT_MIN_DEG              -87.0f
#endif
#ifndef ROT_MAX_DEG
#define ROT_MAX_DEG               97.0f
#endif
#ifndef LOW_MIN_DEG
#define LOW_MIN_DEG              -28.0f
#endif
#ifndef LOW_MAX_DEG
#define LOW_MAX_DEG               87.0f
#endif
#ifndef HIGH_MIN_DEG
#define HIGH_MIN_DEG             -77.0f
#endif
#ifndef HIGH_MAX_DEG
#define HIGH_MAX_DEG               2.0f
#endif

/* HomePose is the fixed pose reached after homing, backoff and encoder zero.
 * Encoder angles are measured on the motor side of the 4.5:1 reduction. */
#ifndef HOMEPOSE_X_MM
#define HOMEPOSE_X_MM              0.0f
#endif
#ifndef HOMEPOSE_Y_MM
#define HOMEPOSE_Y_MM            185.0f
#endif
#ifndef HOMEPOSE_Z_MM
#define HOMEPOSE_Z_MM            240.0f
#endif
#ifndef HOMEPOSE_ROT_DEG
#define HOMEPOSE_ROT_DEG           0.0f
#endif
#ifndef HOMEPOSE_LOW_DEG
#define HOMEPOSE_LOW_DEG           0.0f
#endif
#ifndef HOMEPOSE_HIGH_DEG
#define HOMEPOSE_HIGH_DEG          0.0f
#endif

/* Extra allowance for encoder quantization and mechanical compliance. */
#ifndef ACTUAL_JOINT_LIMIT_TOLERANCE_DEG
#define ACTUAL_JOINT_LIMIT_TOLERANCE_DEG  1.0f
#endif

/* 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
 * Motion Planner / 杞ㄨ抗瑙勫垝�?
 * 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?*/

#ifndef TICKS_PER_MS
#define TICKS_PER_MS            50U          /* TIM6 ticks per ms (50 kHz) / 姣忔绉掑畾鏃跺櫒鑺傛媿�?*/
#endif

#ifndef LINEAR_SEGMENT_MM
#define LINEAR_SEGMENT_MM       0.25f        /* Max segment length (mm) / 鏈€澶у垎娈甸暱搴?*/
#endif

#ifndef PLANNER_SEGMENT_MS
#define PLANNER_SEGMENT_MS      4U           /* Target segment duration (ms) / 鐩爣鍒嗘鏃堕�?*/
#endif

#ifndef MIN_FRAME_TICKS
#define MIN_FRAME_TICKS         1U           /* Minimum ticks per frame / 姣忓抚鏈€灏忚妭鎷嶆�?*/
#endif

#ifndef FRAME_STEP_MARGIN
#define FRAME_STEP_MARGIN       3U           /* Step margin to avoid overrun / 姝ユ暟瑁曢噺闃叉孩鍑?*/
#endif

#ifndef STOP_TAIL_EXTRA_TICKS
#define STOP_TAIL_EXTRA_TICKS   2U           /* Extra ticks for end deceleration / 鏈鍑忛€熼澶栬妭鎷?*/
#endif

#ifndef END_SLOW_SEGMENTS
#define END_SLOW_SEGMENTS       2U           /* Number of deceleration segments / 鍑忛€熸鏁伴噺 */
#endif

#ifndef MOTOR_STEPS_PER_SEG
#define MOTOR_STEPS_PER_SEG     10U          /* Max motor steps per segment / 姣忔鏈€澶х數鏈烘�?*/
#endif

#ifndef TELEOP_FRAME_MS
#define TELEOP_FRAME_MS         10U          /* Teleop frame duration (ms), must match PS2 poll / 閬ユ帶甯ф椂闀? 闇€涓嶱S2杞鍖归厤 */
#endif

#ifndef PLANNER_QUEUE_TIMEOUT_MS
#define PLANNER_QUEUE_TIMEOUT_MS 2000U       /* Queue wait timeout; timeout latches a safety stop. */
#endif

#ifndef CL_ENCODER_FAIL_LIMIT
#define CL_ENCODER_FAIL_LIMIT   3U           /* Consecutive encoder failures before emergency stop. */
#endif

/* 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
 * PID Closed-Loop Control / PID 闂幆鎺у�?
 * 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?*/

#ifndef CL_KP
#define CL_KP                   0.12f        /* Proportional gain / 姣斾緥澧炵泭 */
#endif

#ifndef CL_KI
#define CL_KI                   0.03f        /* Integral gain / 绉垎澧炵泭 */
#endif

#ifndef CL_KD
#define CL_KD                   0.0f         /* Derivative gain (off �?amplifies encoder noise) / 寰垎澧炵泭 (鍏抽�?�?鏀惧ぇ缂栫爜鍣ㄥ櫔�? */
#endif

#ifndef CL_INTEGRAL_MAX
#define CL_INTEGRAL_MAX         3.0f         /* Integral windup limit / 绉垎楗卞拰涓婇�?*/
#endif

#ifndef CL_DEADBAND_DEG
#define CL_DEADBAND_DEG         3.0f         /* Deadband (deg) �?stop correction within this band / 姝诲�?�?鍦ㄦ鑼冨洿鍐呭仠姝慨�?*/
#endif

#ifndef CL_UPDATE_MS
#define CL_UPDATE_MS            20U          /* PID update period (ms) / PID 鏇存柊鍛ㄦ湡 */
#endif

#ifndef CL_I_SEP_ERR
#define CL_I_SEP_ERR            2.5f         /* Integral separation threshold (deg) / 绉垎鍒嗙闃堝�?*/
#endif

#ifndef CL_MIN_TICKS
#define CL_MIN_TICKS            100U         /* Minimum correction ticks / 鏈€灏忎慨姝ｈ妭�?*/
#endif

#ifndef CL_SPEED_DIV
#define CL_SPEED_DIV            200U         /* Correction speed divisor / 淇閫熷害闄ゆ�?*/
#endif

#ifndef CL_EMA_ALPHA
#define CL_EMA_ALPHA            0.35f        /* EMA filter coefficient / 鎸囨暟绉诲姩骞冲潎婊ゆ尝绯绘�?*/
#endif

/* Conservative correction: 3-5 deg small error / 淇濆畧淇: 3-5�?灏忚宸?*/
#ifndef CL_OUTPUT_MAX_LO
#define CL_OUTPUT_MAX_LO        0.3f         /* Max output for small error (deg) / 灏忚宸渶澶ц緭�?*/
#endif

#ifndef CL_COOLDOWN_LO_MS
#define CL_COOLDOWN_LO_MS       200U         /* Cooldown for small error (ms) / 灏忚宸喎鍗存椂闂?*/
#endif

/* Aggressive recovery: >5 deg large error / 寮哄姏鎭㈠: >5�?澶ц�?*/
#ifndef CL_LARGE_ERR_DEG
#define CL_LARGE_ERR_DEG        5.0f         /* Large error threshold (deg) / 澶ц宸槇鍊?*/
#endif

#ifndef CL_OUTPUT_MAX_HI
#define CL_OUTPUT_MAX_HI        4.0f         /* Max output for large error (deg) / 澶ц宸渶澶ц緭鍑?*/
#endif

#ifndef CL_COOLDOWN_HI_MS
#define CL_COOLDOWN_HI_MS       100U         /* Cooldown for large error (ms) / 澶ц宸喎鍗存椂�?*/
#endif

/* 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
 * Static Compensation / 闈欐€佽ˉ�?
 * 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?*/

#ifndef COMP_DEADBAND_DEG
#define COMP_DEADBAND_DEG       2.0f         /* Compensation deadband (deg) / 琛ュ伩姝诲尯 */
#endif

#ifndef COMP_SPEED_DIV
#define COMP_SPEED_DIV          100          /* Compensation speed divisor / 琛ュ伩閫熷害闄ゆ�?*/
#endif

#ifndef COMP_MIN_TICKS
#define COMP_MIN_TICKS          100U         /* Min compensation ticks / 鏈€灏忚ˉ鍋胯妭鎷?*/
#endif

#ifndef COMP_WATCHDOG_ROUNDS
#define COMP_WATCHDOG_ROUNDS    30           /* Max compensation iterations / 鏈€澶цˉ鍋胯凯浠ｆ鏁?*/
#endif

#ifndef COMP_WAIT_TIMEOUT_MS
#define COMP_WAIT_TIMEOUT_MS    3000U        /* Maximum wait for one compensation frame. */
#endif

/* 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
 * Homing / 鍥為�?
 * 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?*/

#ifndef HOMING_STEP_DELAY_US
#define HOMING_STEP_DELAY_US    800          /* Homing step interval (us): 1/4 of the previous seek/backoff speed. */
#endif

#ifndef HOMING_MAX_STEPS
#define HOMING_MAX_STEPS        15000         /* Max steps before timeout / 瓒呮椂鍓嶆渶澶ф�?*/
#endif

#ifndef HOMING_BACKOFF_M1_DEG
#define HOMING_BACKOFF_M1_DEG   100.0f       /* M1 backoff angle (deg) �?joint / M1 鍥為€€瑙掑�?�?鍏宠�?*/
#endif

#ifndef HOMING_BACKOFF_M2_DEG
#define HOMING_BACKOFF_M2_DEG   31.0f        /* M2 backoff angle (deg) �?joint / M2 鍥為€€瑙掑�?�?鍏宠�?*/
#endif

#ifndef HOMING_BACKOFF_M3_DEG
#define HOMING_BACKOFF_M3_DEG   5.0f        /* M3 backoff angle (deg) �?joint / M3 鍥為€€瑙掑�?�?鍏宠�?*/
#endif

/* 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
 * G-code Executor / G-code 鎵ц�?
 * 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?*/

#ifndef GCODE_DEFAULT_FEEDRATE
#define GCODE_DEFAULT_FEEDRATE  3000.0f      /* Default feedrate (mm/min) / 榛樿杩涚粰閫熷�?*/
#endif

#ifndef GCODE_MAX_FEEDRATE
#define GCODE_MAX_FEEDRATE      3000.0f      /* Cartesian safety ceiling (mm/min). */
#endif

#ifndef MOTOR_MAX_STEP_RATE_HZ
#define MOTOR_MAX_STEP_RATE_HZ  10000U       /* Per-axis STEP ceiling (pulses/s). */
#endif

#ifndef GCODE_MIN_MOVE_MM
#define GCODE_MIN_MOVE_MM       0.001f       /* Minimum move distance (mm) / 鏈€灏忕Щ鍔ㄨ窛绂?*/
#endif

#ifndef GCODE_MIN_DURATION_MS
#define GCODE_MIN_DURATION_MS   1U           /* Minimum move duration (ms) / 鏈€灏忕Щ鍔ㄦ椂闀?*/
#endif

/* 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
 * PS2 Teleop / PS2 閬ユ�?
 * 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?*/

#ifndef PS2_POLL_INTERVAL_MS
#define PS2_POLL_INTERVAL_MS    10U          /* Polling interval (ms) / 杞闂撮殧 */
#endif

#ifndef JOYSTICK_DEADZONE
#define JOYSTICK_DEADZONE       12           /* Joystick deadzone / 鎽囨潌姝诲尯 */
#endif

#ifndef TELEOP_MAX_STEP_MM
#define TELEOP_MAX_STEP_MM      0.9f         /* Max step per teleop frame (mm) / 姣忛仴鎺у抚鏈€澶ф闀?*/
#endif

/* 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
 * Gripper Servo / 澶圭埅鑸垫満
 * 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?*/

#ifndef GRIPPER_SERVO_MIN_US
#define GRIPPER_SERVO_MIN_US    500.0f       /* Min pulse width (us) / 鏈€灏忚剦�?*/
#endif

#ifndef GRIPPER_SERVO_MAX_US
#define GRIPPER_SERVO_MAX_US    2500.0f      /* Max pulse width (us) / 鏈€澶ц剦�?*/
#endif

#ifndef GRIPPER_ANGLE_OPEN
#define GRIPPER_ANGLE_OPEN      72.0f        /* Close angle (deg) / 鍚堢埅瑙掑害 */
#endif

#ifndef GRIPPER_ANGLE_CLOSE
#define GRIPPER_ANGLE_CLOSE     100.0f       /* Open angle (deg) / 寮犲紑瑙掑害 */
#endif

#ifndef GRIPPER_HOLD_MS
#define GRIPPER_HOLD_MS         600U         /* PWM hold time before auto-off (ms) / PWM淇濇寔鏃堕棿鍚庤嚜鍔ㄥ叧�?*/
#endif


/* 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
 * TMC2209 Driver / TMC2209 椹卞�?
 * 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?*/

#ifndef TMC_IRUN_HIGH
#define TMC_IRUN_HIGH           24U          /* High run current / 楂樿繍琛岀數娴?*/
#endif

#ifndef TMC_IRUN_MED
#define TMC_IRUN_MED            16U          /* Medium run current / 涓繍琛岀數娴?*/
#endif

#ifndef TMC_IRUN_LOW
#define TMC_IRUN_LOW            8U           /* Low run current / 浣庤繍琛岀數娴?*/
#endif

#ifndef TMC_IHOLD_STRONG
#define TMC_IHOLD_STRONG        12U          /* Strong hold current / 寮轰繚鎸佺數�?*/
#endif

#ifndef TMC_IHOLD_COOL
#define TMC_IHOLD_COOL          4U           /* Low hold current / 浣庝繚鎸佺數�?*/
#endif

#ifndef TMC_DEFAULT_MICROSTEPS
#define TMC_DEFAULT_MICROSTEPS  16U          /* Default microstepping / 榛樿缁嗗垎 */
#endif

#ifndef TMC_DEFAULT_IRUN
#define TMC_DEFAULT_IRUN        28U          /* Default run current / 榛樿杩愯鐢垫�?*/
#endif

#ifndef TMC_DEFAULT_IHOLD
#define TMC_DEFAULT_IHOLD       15U          /* Default hold current / 榛樿淇濇寔鐢垫�?*/
#endif

/* 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
 * System / Timing / 绯荤粺涓庡畾�?
 * 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?*/

#ifndef RING_BUFFER_SIZE
#define RING_BUFFER_SIZE        64           /* Motion frame ring buffer size / 杩愬姩甯х幆褰㈢紦鍐插尯澶у�?*/
#endif

#ifndef UART1_RX_BUF_SIZE
#define UART1_RX_BUF_SIZE       256U         /* UART1 RX buffer size (bytes) / 涓插彛鎺ユ敹缂撳啿鍖哄ぇ�?*/
#endif

#ifndef UART1_TX_BUF_SIZE
#define UART1_TX_BUF_SIZE       1024U        /* UART1 TX DMA queue (bytes). */
#endif

#ifndef UART1_LINE_TIMEOUT_MS
#define UART1_LINE_TIMEOUT_MS   500U         /* Incomplete-line discard timeout (ms). */
#endif


#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_CONFIG_H__ */
