#ifndef ROBOTARM_BSP_DRIVER_CONFIG_H
#define ROBOTARM_BSP_DRIVER_CONFIG_H

/* Gripper servo parameters. */
#ifndef GRIPPER_SERVO_MIN_US
#define GRIPPER_SERVO_MIN_US 500.0f
#endif
#ifndef GRIPPER_SERVO_MAX_US
#define GRIPPER_SERVO_MAX_US 2500.0f
#endif
#ifndef GRIPPER_ANGLE_OPEN
#define GRIPPER_ANGLE_OPEN 72.0f
#endif
#ifndef GRIPPER_ANGLE_CLOSE
#define GRIPPER_ANGLE_CLOSE 100.0f
#endif
#ifndef GRIPPER_HOLD_MS
#define GRIPPER_HOLD_MS 600U
#endif

/* TMC2209 electrical configuration. */
#ifndef TMC_IRUN_HIGH
#define TMC_IRUN_HIGH 24U
#endif
#ifndef TMC_IHOLD_STRONG
#define TMC_IHOLD_STRONG 12U
#endif
#ifndef TMC_DEFAULT_MICROSTEPS
#define TMC_DEFAULT_MICROSTEPS 16U
#endif
#ifndef TMC_DEFAULT_IRUN
#define TMC_DEFAULT_IRUN 28U
#endif
#ifndef TMC_DEFAULT_IHOLD
#define TMC_DEFAULT_IHOLD 15U
#endif

#endif /* ROBOTARM_BSP_DRIVER_CONFIG_H */
