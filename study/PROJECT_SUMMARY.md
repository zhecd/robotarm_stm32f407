# F407AS5600_2 Project Summary

## 1. Project Positioning

This project is a robotic arm control system based on `STM32F407`.

It currently supports two main control modes:

- `G-code mode`: a host computer sends motion commands over UART.
- `PS2 teleop mode`: a PS2 controller drives the robotic arm directly.

The codebase is organized around a clear control chain:

`Input -> Parse/Decision -> Motion Planning -> Motor Scheduling -> Hardware Output`

## 2. Core Functional Architecture

### 2.1 Application Layer

- [main.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/main.c)
  - System startup entry.
  - Initializes GPIO, timers, UART, stepper drivers, PS2, gripper, planner, and executor.
  - Runs the main loop.
  - Dispatches work by current system mode.

- [app_teleop.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/app_teleop.c)
  - PS2 teleoperation application task.
  - Handles controller polling, mode switching, joystick decoding, and gripper button logic.
  - Converts joystick intent into small Cartesian increments.

- [cmd_executor.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/cmd_executor.c)
  - Executes already-parsed G-code frames.
  - Maintains current Cartesian position and current feedrate.
  - Converts G0/G1 into motion planner calls.
  - Converts M3/M5 into gripper actions.

### 2.2 Protocol and Parsing Layer

- [gcode_parser.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/gcode_parser.c)
  - Parses one line of input text into a `GCodeFrame_t`.
  - Supports:
    - `G0`
    - `G1`
    - `M3`
    - `M5`
    - axes `X/Y/Z`
    - feedrate `F`

- [bsp_uart1.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/bsp_uart1.c)
  - UART1 receive ring buffer.
  - Interrupt-driven byte reception.
  - Line extraction with newline or timeout policy.
  - Also provides the serial output path used by `printf`.

### 2.3 Motion Planning Layer

- [motion_planner.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/motion_planner.c)
  - Core Cartesian motion planner.
  - Handles straight-line interpolation in Cartesian space.
  - Uses segmented motion frames and smoothed progress (`Quintic_Smoothstep`) for better start/stop behavior.
  - Provides a separate fast teleop step path for joystick control.

- [robotGeometry.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/robotGeometry.c)
  - Inverse kinematics and geometry conversion layer.
  - Converts Cartesian `(x, y, z)` target positions into joint angles.
  - Converts joint angles into motor units for the three axes.

### 2.4 Motion Execution Layer

- [motor_core.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/motor_core.c)
  - Real-time motion frame scheduler.
  - Owns the motion ring buffer.
  - Runs from `TIM6` interrupt callback.
  - Converts each motion frame into stepped pulse output over time.

This module is the bridge between planning and actual pulse generation.

### 2.5 Hardware Abstraction Layer

- [bsp_stepper.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/bsp_stepper.c)
  - Low-level step/dir control for the three motors.
  - Provides:
    - enable/disable
    - direction set
    - one-step pulse output

- [bsp_tmc2209.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/bsp_tmc2209.c)
  - TMC2209 UART configuration layer.
  - Writes microstep, current, and chopping-related settings to each driver.

- [bsp_ps2.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/bsp_ps2.c)
  - PS2 controller communication driver.
  - Bit-banged protocol implementation using GPIO.
  - Returns button and joystick state.

- [bsp_gripper.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/bsp_gripper.c)
  - Servo gripper abstraction.
  - Supports open, close, custom angle, and stop.

- [bsp_led.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/bsp_led.c)
  - Simple LED status output abstraction.

### 2.6 MCU Peripheral Layer

Generated peripheral configuration files:

- [gpio.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/gpio.c)
- [tim.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/tim.c)
- [usart.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/usart.c)
- [stm32f4xx_it.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/stm32f4xx_it.c)

These files provide the MCU-level plumbing used by the upper layers.

## 3. Main Runtime Behavior

### 3.1 Startup Sequence

1. HAL and system clock initialization.
2. GPIO, UART, TIM peripheral initialization.
3. BSP module initialization:
   - LEDs
   - steppers
   - UART1
   - PS2
   - gripper
4. TMC2209 node configuration.
5. Motion core, planner, and command executor initialization.
6. Start `TIM6` interrupt for motion execution.
7. Enter infinite main loop.

### 3.2 G-code Mode

1. UART1 receives host commands.
2. A full line is assembled by the ring buffer.
3. `gcode_parser` converts the text command into a frame.
4. `cmd_executor` computes target position and duration.
5. `motion_planner` decomposes the move into motion frames.
6. `motor_core` executes those frames via timer interrupt.
7. `bsp_stepper` outputs physical step pulses.

### 3.3 PS2 Mode

1. `App_Teleop_Task()` polls the controller periodically.
2. Joystick values are converted into small `dx/dy/dz` increments.
3. `Motion_Planner_TeleopStep()` packages the move as a short execution frame.
4. `motor_core` executes it in near real time.
5. Buttons can also trigger mode switching and gripper actions.

## 4. Important Design Characteristics

### 4.1 Strengths

- Good separation between parser, executor, planner, scheduler, and hardware drivers.
- Real-time motor scheduling is isolated in timer interrupt context.
- Supports both scripted control and manual teleoperation.
- Robotic-arm geometry is encapsulated in a dedicated inverse kinematics module.
- TMC2209 driver logic is separated from generic step pulse generation.

### 4.2 Current Engineering Style

The project now follows a more maintainable embedded style:

- public APIs live in headers
- internal helpers stay private in source files
- stateful modules keep their own local state
- configuration values are lifted into named constants
- hardware access is separated from application logic

## 5. Key Files by Responsibility

### Control Entry

- [main.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/main.c)

### Mode Management

- [app_teleop.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/app_teleop.c)
- [app_teleop.h](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Inc/app_teleop.h)

### Command Input and Parsing

- [bsp_uart1.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/bsp_uart1.c)
- [gcode_parser.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/gcode_parser.c)
- [cmd_executor.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/cmd_executor.c)

### Motion System

- [motion_planner.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/motion_planner.c)
- [motor_core.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/motor_core.c)
- [robotGeometry.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/robotGeometry.c)

### Device Drivers

- [bsp_stepper.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/bsp_stepper.c)
- [bsp_tmc2209.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/bsp_tmc2209.c)
- [bsp_ps2.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/bsp_ps2.c)
- [bsp_gripper.c](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/Core/Src/bsp_gripper.c)

## 6. Suggested Next Evolution

If you continue evolving this project, the most natural next upgrades are:

1. introduce a dedicated application state manager instead of exposing `current_sys_mode` directly
2. add explicit workspace and inverse-kinematics validity checks
3. split motion planner configuration into a dedicated config header
4. add structured logging/debug levels
5. introduce unit-testable pure functions for parser and kinematics
6. add host-side tooling for text-to-G-code or path generation

## 7. Study Files

This folder also includes:

- [PROJECT_MINDMAP.md](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/study/PROJECT_MINDMAP.md)
- [PROJECT_FLOWCHART.md](/E:/STM32CubeMX%20TEST/vscode%20text/F407AS5600_2/study/PROJECT_FLOWCHART.md)

They are meant to complement this summary with a quick visual overview.
