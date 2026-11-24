/**
 * @file robot_config.h
 * @brief 机械臂参数：几何、单位、速度、软限位、回零和控制周期。
 * @details 关节角单位 deg，编码器为电机轴 deg；两者相差减速比及 M2/M3 的方向符号。
 */

#ifndef __ROBOT_CONFIG_H__
#define __ROBOT_CONFIG_H__

/* 仅专用审计构建开启：运行真实启动检查，在首次机械动作前锁定。
 * 正常构建默认0，保留自动夹爪初始化/回零；审计不能宣称回零成功。 */
#ifndef ROBOTARM_STARTUP_AUDIT_MODE
#define ROBOTARM_STARTUP_AUDIT_MODE 0
#endif

/* 启动/恢复前先等三路反馈可读；由主循环轮询，禁止等待期间提前回零。 */
#ifndef STARTUP_ENCODER_TIMEOUT_MS
#define STARTUP_ENCODER_TIMEOUT_MS 10000U /* 包含读操作耗时的就绪检查期限，ms。 */
#endif
#ifndef STARTUP_ENCODER_POLL_MS
#define STARTUP_ENCODER_POLL_MS 100U /* 一轮三轴采样的最小间隔，ms。 */
#endif

#ifndef CL_SETTLE_MS
#define CL_SETTLE_MS 250U /* 运动结束到再次计算补偿之间的稳定等待时间，ms。 */
#endif

#ifndef CL_FEEDBACK_MAX_READ_MS
#define CL_FEEDBACK_MAX_READ_MS 100U /* 闭环拒绝耗时达到此值的读取，不用迟到角度补偿。 */
#endif
#if CL_FEEDBACK_MAX_READ_MS < 1 || CL_FEEDBACK_MAX_READ_MS > 2147483647U
#error Invalid closed-loop feedback read deadline
#endif

#ifndef CL_MAX_CORRECTION_DEG
#define CL_MAX_CORRECTION_DEG 2.0f /* 普通 PID 单次修正幅值上限，电机轴 deg。 */
#endif

#ifndef CL_DIVERGENCE_TOLERANCE_DEG
#define CL_DIVERGENCE_TOLERANCE_DEG 0.5f /* 补偿后误差允许增加量，电机轴 deg。 */
#endif

#ifndef CL_DIVERGENCE_LIMIT
#define CL_DIVERGENCE_LIMIT 2U /* 连续误差增大达到此次数后判定发散。 */
#endif

#ifndef CL_RECOVERY_TRIGGER_DEG
#define CL_RECOVERY_TRIGGER_DEG 8.0f /* 启用分段恢复轨迹的误差阈值，电机轴 deg。 */
#endif

#ifndef CL_RECOVERY_TARGET_FRACTION
#define CL_RECOVERY_TARGET_FRACTION 1.0f /* 本轮恢复目标占当前误差的比例。 */
#endif

#ifndef CL_RECOVERY_MAX_DEG
#define CL_RECOVERY_MAX_DEG 30.0f /* 兼容原参数的电机角上限；还要受下方1°关节角分段限制。 */
#endif

/* 教学恢复策略：窗口决定是否允许尝试，不是到位精度；死区仍用CL_DEADBAND_DEG。
 * 编码器在电机侧，比较时必须乘GEAR_RATIO换算，不能把10°电机角当10°关节角。 */
#ifndef CL_RECOVERY_WINDOW_JOINT_DEG
#define CL_RECOVERY_WINDOW_JOINT_DEG 10.0f /* 原目标与实测位置最大允许偏差，关节deg，含边界。 */
#endif
#ifndef CL_RECOVERY_ROUND_JOINT_DEG
#define CL_RECOVERY_ROUND_JOINT_DEG 1.0f /* 每轮最多修正1°关节角，然后重新读取反馈。 */
#endif
#ifndef CL_RECOVERY_TIMEOUT_MS
#define CL_RECOVERY_TIMEOUT_MS 20000U /* 一次恢复从首次超死区到收敛的总期限，包含轨迹/稳定等待。 */
#endif
#ifndef CL_RECOVERY_PROGRESS_FRACTION
#define CL_RECOVERY_PROGRESS_FRACTION 0.2f /* 改善量至少为本轮实际指令角的20%，兼顾短补偿。 */
#endif

#ifndef CL_RECOVERY_MIN_PROGRESS_DEG
#define CL_RECOVERY_MIN_PROGRESS_DEG 2.0f /* 改善判据上限，实际取本轮指令角20%与此值的较小值。 */
#endif

#ifndef CL_RECOVERY_NO_PROGRESS_LIMIT
#define CL_RECOVERY_NO_PROGRESS_LIMIT 3U /* 连续三轮改善不足停止；不无限补偿或自动重试故障。 */
#endif

#ifndef CL_RECOVERY_STEPS_PER_SEGMENT
#define CL_RECOVERY_STEPS_PER_SEGMENT 1U /* 恢复轨迹分段时的参考微步数。 */
#endif

#ifndef CL_RECOVERY_TICKS_PER_STEP
#define CL_RECOVERY_TICKS_PER_STEP 300U /* 恢复轨迹按微步估算时长的节拍系数。 */
#endif

#ifndef CL_RECOVERY_QUEUE_AHEAD
#define CL_RECOVERY_QUEUE_AHEAD 24U /* 恢复轨迹预填帧数量上限。 */
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Motor Parameters */

#ifndef STEPS_PER_REV
#define STEPS_PER_REV           3200.0f /* 电机轴每转微步数，200 整步 × 16 微步。 */
#endif

#ifndef GEAR_RATIO
#define GEAR_RATIO              (90.0f / 20.0f) /* 电机转角/关节转角的传动比。 */
#endif

#ifndef FEEDBACK_QUERY_MAX_AGE_MS
#define FEEDBACK_QUERY_MAX_AGE_MS 100U /* M409逐轴新鲜度的排他上界；不等于运动许可。 */
#endif
#if FEEDBACK_QUERY_MAX_AGE_MS < 1 || FEEDBACK_QUERY_MAX_AGE_MS > 2147483647U
#error Invalid feedback query age bound
#endif

#ifndef STEPS_PER_DEGREE
#define STEPS_PER_DEGREE        (STEPS_PER_REV / 360.0f) /* 每电机轴度对应的微步数。 */
#endif

#ifndef DEGREES_PER_STEP
#define DEGREES_PER_STEP        (360.0f / STEPS_PER_REV) /* 每微步对应的电机轴角度，deg/step。 */
#endif

/*
 * Kinematics (SCARA-like parallel-gripper arm) */

#ifndef LINK_1_LEN
#define LINK_1_LEN              140.0f /* 大臂长度，mm。 */
#endif

#ifndef LINK_2_LEN
#define LINK_2_LEN              140.0f /* 小臂长度，mm。 */
#endif

#ifndef TOOL_OFFSET_R
#define TOOL_OFFSET_R           52.0f /* 夹爪径向向外偏移，mm。 */
#endif

#ifndef TOOL_OFFSET_Z
#define TOOL_OFFSET_Z           -40.0f /* 夹爪竖直方向偏移，mm。 */
#endif

#ifndef BASE_HEIGHT
#define BASE_HEIGHT             145.0f /* 桌面到肩关节的高度，mm。 */
#endif

#ifndef UNITS_PER_DEGREE
#define UNITS_PER_DEGREE       (STEPS_PER_REV * GEAR_RATIO / 360.0f) /* 每关节度对应的电机微步数，已包含减速比。 */
#endif

#ifndef OFFSET_ROT
#define OFFSET_ROT              0.0f /* 历史零偏配置项；当前运动学未读取此项。 */
#endif

#ifndef OFFSET_LOW
#define OFFSET_LOW              0.0f /* 历史零偏配置项；当前运动学未读取此项。 */
#endif

#ifndef OFFSET_HIGH
#define OFFSET_HIGH             0.0f /* 历史零偏配置项；当前运动学未读取此项。 */
#endif

/* Software joint limits.  Fill these values from measured safe joint ranges
 * after homing; do not enable this feature with guessed mechanical limits. */
#ifndef JOINT_LIMITS_ENABLED
#define JOINT_LIMITS_ENABLED     1U /* 是否启用规划关节软限位检查。 */
#endif

#ifndef ROT_MIN_DEG
#define ROT_MIN_DEG              -87.0f /* 关节角允许范围边界，deg。 */
#endif
#ifndef ROT_MAX_DEG
#define ROT_MAX_DEG               97.0f /* 关节角允许范围边界，deg。 */
#endif
#ifndef LOW_MIN_DEG
#define LOW_MIN_DEG              -25.0f /* 关节角允许范围边界，deg。 */
#endif
#ifndef LOW_MAX_DEG
#define LOW_MAX_DEG               93.0f /* 关节角允许范围边界，deg。 */
#endif
#ifndef HIGH_MIN_DEG
#define HIGH_MIN_DEG             -100.0f /* 关节角允许范围边界，deg。 */
#endif
#ifndef HIGH_MAX_DEG
#define HIGH_MAX_DEG               2.0f /* 关节角允许范围边界，deg。 */
#endif

/* HomePose is the fixed pose reached after homing, backoff and encoder zero.
 * Encoder angles are measured on the motor side of the 4.5:1 reduction. */
#ifndef HOMEPOSE_X_MM
#define HOMEPOSE_X_MM              0.0f /* 回零后参考姿态，XYZ 为 mm、其余为关节 deg。 */
#endif
#ifndef HOMEPOSE_Y_MM
#define HOMEPOSE_Y_MM            (LINK_2_LEN + TOOL_OFFSET_R) /* 三关节为 0° 时的工具点径向坐标，mm。 */
#endif
#ifndef HOMEPOSE_Z_MM
#define HOMEPOSE_Z_MM            (BASE_HEIGHT + LINK_1_LEN + TOOL_OFFSET_Z) /* 三关节为 0° 时的工具点高度，mm。 */
#endif
#ifndef HOMEPOSE_ROT_DEG
#define HOMEPOSE_ROT_DEG           0.0f /* 回零后参考姿态，XYZ 为 mm、其余为关节 deg。 */
#endif
#ifndef HOMEPOSE_LOW_DEG
#define HOMEPOSE_LOW_DEG           0.0f /* 回零后参考姿态，XYZ 为 mm、其余为关节 deg。 */
#endif
#ifndef HOMEPOSE_HIGH_DEG
#define HOMEPOSE_HIGH_DEG          0.0f /* 回零后参考姿态，XYZ 为 mm、其余为关节 deg。 */
#endif

/* Extra allowance for encoder quantization and mechanical compliance. */
#ifndef ACTUAL_JOINT_LIMIT_TOLERANCE_DEG
#define ACTUAL_JOINT_LIMIT_TOLERANCE_DEG  1.0f /* 实际反馈角判断时的限位容差，关节 deg。 */
#endif

/*
 * Motion Planner */

#ifndef TICKS_PER_MS
#define TICKS_PER_MS            50U /* STEP 定时器每毫秒节拍数，50 对应 50 kHz。 */
#endif

#ifndef LINEAR_SEGMENT_MM
#define LINEAR_SEGMENT_MM       0.25f /* 规划的空间分段参考长度，mm。 */
#endif

#ifndef PLANNER_SEGMENT_MS
#define PLANNER_SEGMENT_MS      20U /* 规划的时间分段参考长度，ms。 */
#endif

#ifndef MIN_FRAME_TICKS
#define MIN_FRAME_TICKS         1U           /* Minimum ticks per frame */
#endif

#ifndef FRAME_STEP_MARGIN
#define FRAME_STEP_MARGIN       3U           /* Step margin to avoid overrun */
#endif

#ifndef STOP_TAIL_EXTRA_TICKS
#define STOP_TAIL_EXTRA_TICKS   2U           /* Extra ticks for end deceleration */
#endif

#ifndef END_SLOW_SEGMENTS
#define END_SLOW_SEGMENTS       2U           /* Number of deceleration segments */
#endif

#ifndef MOTOR_STEPS_PER_SEG
#define MOTOR_STEPS_PER_SEG     10U          /* Max motor steps per segment */
#endif

#ifndef TELEOP_FRAME_MS
#define TELEOP_FRAME_MS         10U /* 遥操作运动帧的基准时长，ms。 */
#endif

#ifndef PLANNER_QUEUE_TIMEOUT_MS
#define PLANNER_QUEUE_TIMEOUT_MS 2000U /* 帧队列持续无法提交的等待上限，ms；超时锁存故障。 */
#endif

#ifndef CL_ENCODER_FAIL_LIMIT
#define CL_ENCODER_FAIL_LIMIT   3U /* 连续读取失败达到此次数后锁存编码器故障。 */
#endif

/*
 * PID Closed-Loop Control */

#ifndef CL_KP
#define CL_KP                   0.12f /* 比例增益。 */
#endif

#ifndef CL_KI
#define CL_KI                   0.03f /* 积分增益。 */
#endif

#ifndef CL_KD
#define CL_KD                   0.0f /* 微分增益；当前为 0。 */
#endif

#ifndef CL_INTEGRAL_MAX
#define CL_INTEGRAL_MAX         3.0f /* 积分累积幅值上限。 */
#endif

#ifndef CL_DEADBAND_DEG
#define CL_DEADBAND_DEG         3.0f /* 电机轴角度误差死区，deg。 */
#endif

#ifndef CL_UPDATE_MS
#define CL_UPDATE_MS            20U /* 闭环计算周期，ms。 */
#endif

#ifndef CL_I_SEP_ERR
#define CL_I_SEP_ERR            2.5f /* 参与积分计算的电机轴角度误差阈值，deg。 */
#endif

#ifndef CL_MIN_TICKS
#define CL_MIN_TICKS            100U /* 补偿帧最小节拍数。 */
#endif

#ifndef CL_SPEED_DIV
#define CL_SPEED_DIV            200U /* 保留的兼容配置项，当前补偿路径未使用。 */
#endif

#ifndef CL_EMA_ALPHA
#define CL_EMA_ALPHA            0.35f /* 编码器读数的指数移动平均滤波系数。 */
#endif

/* Conservative correction: 3-5 deg small error */
#ifndef CL_OUTPUT_MAX_LO
#define CL_OUTPUT_MAX_LO        0.3f /* 小误差 PID 输出上限，电机轴 deg。 */
#endif

#ifndef CL_COOLDOWN_LO_MS
#define CL_COOLDOWN_LO_MS       200U /* 小误差补偿冷却时间，ms。 */
#endif

/* Aggressive recovery: >5 deg large error */
#ifndef CL_LARGE_ERR_DEG
#define CL_LARGE_ERR_DEG        5.0f /* 区分大小误差的阈值，电机轴 deg。 */
#endif

#ifndef CL_OUTPUT_MAX_HI
#define CL_OUTPUT_MAX_HI        4.0f /* 大误差 PID 输出上限，电机轴 deg。 */
#endif

#ifndef CL_COOLDOWN_HI_MS
#define CL_COOLDOWN_HI_MS       100U /* 大误差补偿冷却时间，ms。 */
#endif

/*
 * Static Compensation */

/*
 * Homing */

/* DIR引脚实际高低电平，不把它命名为物理顺/逆时针。
 * M1 D0已在39/45/48实测；M2 D0由56手动趋近与51/58的STEP阶段趋势支持；
 * M3 D1由56手动趋近与60点动核对支持。M2/M3的HOME尚未实测。
 * 服务须调用SetDirectionLevel，不可传给兼容的clockwise接口。
 * 退回方向必须从此处取反，禁止在服务内部维护另一份硬编码方向。 */
#ifndef HOMING_SEEK_M1_DIR_HIGH
#define HOMING_SEEK_M1_DIR_HIGH 0
#endif
#ifndef HOMING_SEEK_M2_DIR_HIGH
#define HOMING_SEEK_M2_DIR_HIGH 0
#endif
#ifndef HOMING_SEEK_M3_DIR_HIGH
#define HOMING_SEEK_M3_DIR_HIGH 1
#endif
#if (HOMING_SEEK_M1_DIR_HIGH != 0 && HOMING_SEEK_M1_DIR_HIGH != 1) || \
    (HOMING_SEEK_M2_DIR_HIGH != 0 && HOMING_SEEK_M2_DIR_HIGH != 1) || \
    (HOMING_SEEK_M3_DIR_HIGH != 0 && HOMING_SEEK_M3_DIR_HIGH != 1)
#error Homing DIR levels must be 0 or 1
#endif

#ifndef HOMING_STEP_DELAY_US
#define HOMING_STEP_DELAY_US    800 /* 回零寻限位和退回的最小步间隔，us。 */
#endif

/* 正式GPIO采样回零使用独立毫秒周期，旧HOMING_STEP_DELAY_US仅用于历史入口。
 * 1ms对应1000微步/秒；任务迟到时只发一个请求，不补发欠步。
 * 不改变10ms消抖、退回角度或220秒整轮故障截止。 */
#ifndef HOMING_SEQUENCE_SEEK_PERIOD_MS
#define HOMING_SEQUENCE_SEEK_PERIOD_MS 1U
#endif
#ifndef HOMING_SEQUENCE_BACKOFF_PERIOD_MS
#define HOMING_SEQUENCE_BACKOFF_PERIOD_MS 1U
#endif

#ifndef HOMING_MAX_STEPS
#define HOMING_MAX_STEPS        15000 /* 寻限位的最大推进轮数，超过后判定失败。 */
#endif

#ifndef HOMING_BACKOFF_M1_DEG
#if ROBOTARM_PARALLEL_HOMING && !ROBOTARM_STARTUP_TRIAL_MODE
#define HOMING_BACKOFF_M1_DEG   101.7f /* M1 回零退回关节角，deg。 */
#else
#define HOMING_BACKOFF_M1_DEG   100.0f /* M1 回零退回关节角，deg。 */
#endif
#endif

#ifndef HOMING_BACKOFF_M2_DEG
#define HOMING_BACKOFF_M2_DEG   31.0f /* M2 回零退回关节角，deg。 */
#endif

#ifndef HOMING_BACKOFF_M3_DEG
#define HOMING_BACKOFF_M3_DEG   5.0f /* M3 回零退回关节角，deg。 */
#endif

/* 正式并行回零的M2退回量；旧逐轴及台架试验继续使用31°。 */
#ifndef HOMING_SEQUENCE_BACKOFF_M2_DEG
#if ROBOTARM_PARALLEL_HOMING && !ROBOTARM_STARTUP_TRIAL_MODE
#define HOMING_SEQUENCE_BACKOFF_M2_DEG 29.0f /* 1160微步。 */
#else
#define HOMING_SEQUENCE_BACKOFF_M2_DEG HOMING_BACKOFF_M2_DEG
#endif
#endif

/* GPIO回零的M3退回量：40微步/关节度；正式并行4°，旧逐轴10°。
 * 启动已压住时从当前位置退回；退足步数后仍须稳定释放，不能仅凭步数发布零位。
 * 旧回零实现继续使用上面的5°参数，便于教学对照和回滚。 */
#ifndef HOMING_SEQUENCE_BACKOFF_M3_DEG
#if ROBOTARM_PARALLEL_HOMING && !ROBOTARM_STARTUP_TRIAL_MODE
#define HOMING_SEQUENCE_BACKOFF_M3_DEG 4.0f /* 正式并行回零：M3 退回 160 微步。 */
#else
#define HOMING_SEQUENCE_BACKOFF_M3_DEG 10.0f /* 保留旧逐轴及台架试验配置。 */
#endif
#endif

/*
 * G-code Executor */

#ifndef GCODE_DEFAULT_FEEDRATE
#define GCODE_DEFAULT_FEEDRATE  3000.0f /* 缺省笛卡尔进给速度，mm/min。 */
#endif

#ifndef GCODE_MAX_FEEDRATE
#define GCODE_MAX_FEEDRATE      3000.0f /* 允许的笛卡尔速度上限，mm/min。 */
#endif

#ifndef MOTOR_MAX_STEP_RATE_HZ
#define MOTOR_MAX_STEP_RATE_HZ  10000U /* 每轴 STEP 速率上限，pulse/s。 */
#endif

#ifndef GCODE_MIN_MOVE_MM
#define GCODE_MIN_MOVE_MM       0.001f /* 小于此距离按零长度请求处理，mm。 */
#endif

#ifndef GCODE_MIN_DURATION_MS
#define GCODE_MIN_DURATION_MS   1U /* 非零运动的最小请求时长，ms。 */
#endif

#ifndef COMMAND_WAYPOINT_QUEUE_SIZE
#define COMMAND_WAYPOINT_QUEUE_SIZE 16U /* 待处理坐标 FIFO 可用容量，另分配 1 槽区分空满。 */
#endif

/*
 * PS2 Teleop */

#ifndef PS2_POLL_INTERVAL_MS
#define PS2_POLL_INTERVAL_MS    10U /* 手柄轮询周期，ms。 */
#endif

#ifndef PS2_INPUT_ACQUIRE_TIMEOUT_MS
#define PS2_INPUT_ACQUIRE_TIMEOUT_MS 750U /* 初次模拟模式协商的最长等待；期间不接受动作。 */
#endif

#ifndef JOYSTICK_DEADZONE
#define JOYSTICK_DEADZONE       12 /* 摇杆以 128 为中心的原始值死区。 */
#endif

#ifndef TELEOP_MAX_STEP_MM
#define TELEOP_MAX_STEP_MM      0.9f /* 每次遥操作请求的最大坐标增量，mm。 */
#endif


/*
 * TMC2209 Driver */

/* System */

#ifndef RING_BUFFER_SIZE
#define RING_BUFFER_SIZE        64 /* 帧队列存储槽数；留空 1 槽，实际可排队 63 帧。 */
#endif


#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_CONFIG_H__ */
