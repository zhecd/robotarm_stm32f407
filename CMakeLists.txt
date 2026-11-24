cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

option(ROBOTARM_HOST_TESTS_ONLY "Build native tests without STM32 toolchain" OFF)
if(ROBOTARM_HOST_TESTS_ONLY)
    project(robotarm_host C)
    enable_testing()
    add_subdirectory(tests)
    return()
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME robotarm_stm32f407)
include("cmake/gcc-arm-none-eabi.cmake")
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME} C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

option(ROBOTARM_UART_BOOT_PROBE "Isolated boot/heartbeat USART1 probe; no robot application" OFF)
if(ROBOTARM_UART_BOOT_PROBE)
    # 专用入口不能与任何业务/动作候选混编；仅链接CMSIS启动和本目录。
    foreach(mode ROBOTARM_DIAGNOSTIC_MODE ROBOTARM_WAVEFORM_MODE
                 ROBOTARM_LIMIT_PROBE_MODE ROBOTARM_TMC_BENCH_MODE
                 ROBOTARM_TMC_CONFIG_PROBE_MODE ROBOTARM_STARTUP_AUDIT_MODE
                 ROBOTARM_STARTUP_TRIAL_MODE ROBOTARM_SEQUENTIAL_HOMING
                 ROBOTARM_LIMIT_STATE_MODE ROBOTARM_PARALLEL_HOMING
                 ROBOTARM_GRIPPER_TEST_MODE GRIPPER_TEST_MOTION_ENABLED
                 ROBOTARM_M1_HOME_MODE M1_HOME_MOTION_ENABLED STARTUP_TRIAL_MOTION_ENABLED)
        if(${mode})
            message(FATAL_ERROR "UART boot probe must be isolated: ${mode} is enabled")
        endif()
    endforeach()
    add_subdirectory(tools/uart_boot_probe)
    return()
endif()

option(ROBOTARM_DIAGNOSTIC_MODE "Build a read-only board diagnostic firmware" OFF)
option(ROBOTARM_DIAGNOSTIC_ENCODERS "Poll encoders in read-only diagnostics" ON)
option(ROBOTARM_WAVEFORM_MODE "Build bounded STEP waveform firmware with EN disabled" OFF)
option(ROBOTARM_LIMIT_PROBE_MODE "Build isolated RAM-only GPIO/EXTI diagnostic without robot stack" OFF)
option(ROBOTARM_TMC_BENCH_MODE "Build explicit one-shot external-motor UART bench" OFF)
option(ROBOTARM_TMC_CONFIG_PROBE_MODE "Build TMC configuration check with outputs permanently inhibited" OFF)
option(ROBOTARM_STARTUP_AUDIT_MODE "Audit real normal startup up to the actuator boundary, outputs locked" OFF)
option(ROBOTARM_IRQ_EVIDENCE_MODE "Read-only startup audit with bounded pre/post EXTI evidence" OFF)
option(ROBOTARM_STARTUP_TRIAL_MODE "One-shot guarded real startup trial, default output locked" OFF)
option(ROBOTARM_SEQUENTIAL_HOMING "Candidate: real sequential contact/backoff/rearm homing" OFF)
option(ROBOTARM_LIMIT_STATE_MODE "Candidate: 1ms GPIO limit sampling, no limit EXTI" OFF)
option(ROBOTARM_PARALLEL_HOMING "Three-axis contact barrier before parallel backoff" OFF)
option(ROBOTARM_STARTUP_GRIPPER_IDLE "Bind the gripper at startup without commanding it" OFF)
if(ROBOTARM_PARALLEL_HOMING AND NOT (ROBOTARM_SEQUENTIAL_HOMING AND ROBOTARM_LIMIT_STATE_MODE))
    message(FATAL_ERROR "Parallel homing requires the qualified GPIO homing controller")
endif()
if(ROBOTARM_LIMIT_STATE_MODE AND NOT ROBOTARM_SEQUENTIAL_HOMING)
    message(FATAL_ERROR "Limit state sampling requires the sequential homing controller")
endif()
if(ROBOTARM_SEQUENTIAL_HOMING AND (ROBOTARM_DIAGNOSTIC_MODE OR ROBOTARM_WAVEFORM_MODE OR ROBOTARM_LIMIT_PROBE_MODE OR ROBOTARM_TMC_BENCH_MODE OR ROBOTARM_TMC_CONFIG_PROBE_MODE))
    message(FATAL_ERROR "Sequential homing is only for normal startup, output audit or its guarded startup trial")
endif()
if(ROBOTARM_IRQ_EVIDENCE_MODE AND NOT (ROBOTARM_STARTUP_AUDIT_MODE OR ROBOTARM_STARTUP_TRIAL_MODE))
    message(FATAL_ERROR "IRQ evidence requires an output-inhibited audit or explicit one-shot trial")
endif()
option(STARTUP_TRIAL_MOTION_ENABLED "Compile explicit one-shot whole-startup motion candidate" OFF)
set(STARTUP_TRIAL_READY_HOLD_MS "0" CACHE STRING "Guarded READY hold window 0..10000ms; only explicit GPIO trial")
if(NOT STARTUP_TRIAL_READY_HOLD_MS MATCHES "^(0|[1-9][0-9]*)$" OR STARTUP_TRIAL_READY_HOLD_MS GREATER 10000)
    message(FATAL_ERROR "READY hold must be an integer 0..10000ms")
endif()
if(STARTUP_TRIAL_READY_HOLD_MS GREATER 0 AND NOT (ROBOTARM_STARTUP_TRIAL_MODE AND ROBOTARM_SEQUENTIAL_HOMING AND ROBOTARM_LIMIT_STATE_MODE))
    message(FATAL_ERROR "READY hold requires the explicit GPIO sequential startup trial")
endif()
option(ROBOTARM_LIMIT_SEEK_TRIAL "Sequential bounded contact trial; no backoff or gripper" OFF)
option(LIMIT_SEEK_UNTIL_CONTACT "Extended contact search with 30s per-axis fault timeout" OFF)
if(LIMIT_SEEK_UNTIL_CONTACT AND NOT ROBOTARM_LIMIT_SEEK_TRIAL)
    message(FATAL_ERROR "Extended contact search requires limit seek trial")
endif()
if(ROBOTARM_LIMIT_SEEK_TRIAL AND NOT ROBOTARM_STARTUP_TRIAL_MODE)
    message(FATAL_ERROR "Limit seek trial requires startup trial safety shell")
endif()
if(ROBOTARM_SEQUENTIAL_HOMING AND ROBOTARM_LIMIT_SEEK_TRIAL)
    message(FATAL_ERROR "Do not combine the production homing candidate with the old contact-only trial")
endif()
if(STARTUP_TRIAL_MOTION_ENABLED AND NOT ROBOTARM_STARTUP_TRIAL_MODE)
    message(FATAL_ERROR "Whole-startup motion requires the dedicated trial mode")
endif()
option(ROBOTARM_GRIPPER_TEST_MODE "Build isolated gripper test with joints permanently inhibited" OFF)
option(GRIPPER_TEST_MOTION_ENABLED "Compile one-shot gripper PWM; separate on-site approval required" OFF)
option(ROBOTARM_M1_HOME_MODE "Build isolated bounded single-axis M1 preparation firmware" OFF)
option(M1_HOME_MOTION_ENABLED "Compile permission for M1 motion; requires separate on-site approval" OFF)
option(M1_JOG_MOTOR_TEN_DEG "Explicit M1 test profile: 89 microsteps, 2000ms deadline" OFF)
option(M1_JOG_JOINT_TEN_DEG "Explicit M1 test profile: 400 microsteps, 9000ms deadline" OFF)
if(ROBOTARM_SEQUENTIAL_HOMING AND (ROBOTARM_M1_HOME_MODE OR ROBOTARM_GRIPPER_TEST_MODE))
    message(FATAL_ERROR "Sequential production homing cannot use isolated single-axis/gripper entry points")
endif()
if(ROBOTARM_STARTUP_TRIAL_MODE AND (ROBOTARM_STARTUP_AUDIT_MODE OR ROBOTARM_GRIPPER_TEST_MODE OR ROBOTARM_M1_HOME_MODE OR ROBOTARM_TMC_CONFIG_PROBE_MODE OR ROBOTARM_TMC_BENCH_MODE OR ROBOTARM_LIMIT_PROBE_MODE OR ROBOTARM_DIAGNOSTIC_MODE OR ROBOTARM_WAVEFORM_MODE OR M1_HOME_MOTION_ENABLED OR GRIPPER_TEST_MOTION_ENABLED OR M1_JOG_MOTOR_TEN_DEG OR M1_JOG_JOINT_TEN_DEG))
    message(FATAL_ERROR "Whole-startup trial cannot combine with other actuator or diagnostic entry points")
endif()
set(SINGLE_AXIS_ID "1" CACHE STRING "Isolated test axis: 1=M1, 2=M2, 3=M3; never enables motion")
set_property(CACHE SINGLE_AXIS_ID PROPERTY STRINGS 1 2 3)
if(NOT SINGLE_AXIS_ID MATCHES "^[123]$")
    message(FATAL_ERROR "SINGLE_AXIS_ID must be 1, 2 or 3")
endif()
if(NOT SINGLE_AXIS_ID STREQUAL "1" AND (M1_JOG_MOTOR_TEN_DEG OR M1_JOG_JOINT_TEN_DEG))
    message(FATAL_ERROR "Larger M1 profiles cannot be used for unverified M2/M3")
endif()
if(M1_JOG_MOTOR_TEN_DEG AND M1_JOG_JOINT_TEN_DEG)
    message(FATAL_ERROR "Choose motor-angle OR joint-angle M1 profile, never both")
endif()
if(ROBOTARM_GRIPPER_TEST_MODE)
    if(ROBOTARM_M1_HOME_MODE OR ROBOTARM_STARTUP_AUDIT_MODE OR ROBOTARM_TMC_CONFIG_PROBE_MODE OR ROBOTARM_TMC_BENCH_MODE OR ROBOTARM_LIMIT_PROBE_MODE OR ROBOTARM_DIAGNOSTIC_MODE OR ROBOTARM_WAVEFORM_MODE OR M1_HOME_MOTION_ENABLED OR M1_JOG_MOTOR_TEN_DEG OR M1_JOG_JOINT_TEN_DEG OR NOT SINGLE_AXIS_ID STREQUAL "1")
        message(FATAL_ERROR "Gripper test must be isolated from all joint/robot applications")
    endif()
    add_subdirectory(tools/gripper_test)
    return()
endif()
if(GRIPPER_TEST_MOTION_ENABLED)
    message(FATAL_ERROR "Gripper motion permission requires the isolated gripper test mode")
endif()
if(ROBOTARM_M1_HOME_MODE)
    if(ROBOTARM_STARTUP_AUDIT_MODE OR ROBOTARM_TMC_CONFIG_PROBE_MODE OR ROBOTARM_TMC_BENCH_MODE OR ROBOTARM_LIMIT_PROBE_MODE OR ROBOTARM_DIAGNOSTIC_MODE OR ROBOTARM_WAVEFORM_MODE)
        message(FATAL_ERROR "M1 homing test must be isolated from all robot applications")
    endif()
    add_subdirectory(tools/m1_homing)
    return()
endif()
if(M1_HOME_MOTION_ENABLED OR M1_JOG_MOTOR_TEN_DEG OR M1_JOG_JOINT_TEN_DEG OR NOT SINGLE_AXIS_ID STREQUAL "1")
    message(FATAL_ERROR "M1 motion permission only applies to isolated M1 firmware")
endif()
if(ROBOTARM_STARTUP_AUDIT_MODE AND (ROBOTARM_TMC_CONFIG_PROBE_MODE OR ROBOTARM_TMC_BENCH_MODE OR ROBOTARM_LIMIT_PROBE_MODE OR ROBOTARM_DIAGNOSTIC_MODE OR ROBOTARM_WAVEFORM_MODE))
    message(FATAL_ERROR "Startup audit cannot be combined with other diagnostic modes")
endif()
if(ROBOTARM_TMC_CONFIG_PROBE_MODE)
    if(ROBOTARM_TMC_BENCH_MODE OR ROBOTARM_LIMIT_PROBE_MODE OR ROBOTARM_DIAGNOSTIC_MODE OR ROBOTARM_WAVEFORM_MODE)
        message(FATAL_ERROR "TMC configuration probe must be isolated from all motion applications")
    endif()
    add_subdirectory(tools/tmc_config_probe)
    return()
endif()
if(ROBOTARM_TMC_BENCH_MODE)
    if(ROBOTARM_LIMIT_PROBE_MODE OR ROBOTARM_DIAGNOSTIC_MODE OR ROBOTARM_WAVEFORM_MODE)
        message(FATAL_ERROR "External-motor bench must be isolated from all robot applications")
    endif()
    add_subdirectory(tools/tmc_bench)
    return()
endif()
if(ROBOTARM_LIMIT_PROBE_MODE AND (ROBOTARM_DIAGNOSTIC_MODE OR ROBOTARM_WAVEFORM_MODE))
    message(FATAL_ERROR "RAM-only probe cannot include diagnostic/waveform application stacks")
endif()
if(ROBOTARM_LIMIT_PROBE_MODE)
    # 独立链接入口，不把自动回零、夹爪或其他业务带进最小对照。
    add_subdirectory(tools/limit_probe)
    return()
endif()

add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

# All hand-written robot firmware lives below this directory.  CubeMX output
# intentionally remains at the project root in Core/ and Drivers/.
set(ROBOTARM_SOURCE_ROOT RobotArm)
if(ROBOTARM_DIAGNOSTIC_MODE AND ROBOTARM_WAVEFORM_MODE)
    message(FATAL_ERROR "Diagnostic and waveform entry points are mutually exclusive")
endif()

set(ROBOTARM_HAL_INCLUDE_DIRS
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
)

function(robotarm_target target)
    target_compile_definitions(${target} PRIVATE ROBOTARM_PARALLEL_HOMING=$<BOOL:${ROBOTARM_PARALLEL_HOMING}>)
    target_compile_definitions(${target} PRIVATE ROBOTARM_STARTUP_GRIPPER_IDLE=$<BOOL:${ROBOTARM_STARTUP_GRIPPER_IDLE}>)
    target_compile_definitions(${target} PRIVATE ROBOTARM_LIMIT_STATE_MODE=$<BOOL:${ROBOTARM_LIMIT_STATE_MODE}>)
    target_compile_definitions(${target} PRIVATE ROBOTARM_SEQUENTIAL_HOMING=$<BOOL:${ROBOTARM_SEQUENTIAL_HOMING}>)
    target_compile_definitions(${target} PRIVATE ROBOTARM_LIMIT_SEEK_TRIAL=$<BOOL:${ROBOTARM_LIMIT_SEEK_TRIAL}>)
    target_compile_definitions(${target} PRIVATE ROBOTARM_IRQ_EVIDENCE_MODE=$<BOOL:${ROBOTARM_IRQ_EVIDENCE_MODE}>)
    target_compile_definitions(${target} PRIVATE ROBOTARM_STARTUP_TRIAL_MODE=$<BOOL:${ROBOTARM_STARTUP_TRIAL_MODE}>)
    if(ROBOTARM_STARTUP_TRIAL_MODE)
        target_compile_definitions(${target} PRIVATE HOMING_STEP_DELAY_US=20000 HOMING_MAX_STEPS=200)
    endif()
    target_compile_definitions(${target} PRIVATE ROBOTARM_STARTUP_AUDIT_MODE=$<BOOL:${ROBOTARM_STARTUP_AUDIT_MODE}>)
    target_compile_definitions(${target} PRIVATE USE_HAL_DRIVER STM32F407xx)
    target_compile_definitions(${target} PRIVATE ROBOTARM_DIAGNOSTIC_MODE=$<BOOL:${ROBOTARM_DIAGNOSTIC_MODE}>)
    target_compile_definitions(${target} PRIVATE ROBOTARM_WAVEFORM_MODE=$<BOOL:${ROBOTARM_WAVEFORM_MODE}>)
    target_compile_definitions(${target} PRIVATE ROBOTARM_DIAGNOSTIC_ENCODERS=$<BOOL:${ROBOTARM_DIAGNOSTIC_ENCODERS}>)
endfunction()
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ROBOTARM_SEQUENTIAL_HOMING=$<BOOL:${ROBOTARM_SEQUENTIAL_HOMING}>)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ROBOTARM_PARALLEL_HOMING=$<BOOL:${ROBOTARM_PARALLEL_HOMING}>)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ROBOTARM_LIMIT_STATE_MODE=$<BOOL:${ROBOTARM_LIMIT_STATE_MODE}>)
set_source_files_properties(RobotArm/Algorithm/common/home_sequence.c
    RobotArm/Service/homing/svc_homing_sequence.c PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror")

# Platform is the only location for bare-metal time, delay and critical-section
# primitives. os_adapter.c remains a temporary compatibility facade.
add_library(arm_platform STATIC
    ${ROBOTARM_SOURCE_ROOT}/Platform/platform_time.c
    ${ROBOTARM_SOURCE_ROOT}/Platform/platform_delay.c
    ${ROBOTARM_SOURCE_ROOT}/Platform/platform_critical.c
    ${ROBOTARM_SOURCE_ROOT}/Platform/compat/os/os_adapter.c
)
robotarm_target(arm_platform)
target_include_directories(arm_platform PRIVATE
    Core/Inc ${ROBOTARM_SOURCE_ROOT}/Platform ${ROBOTARM_SOURCE_ROOT}/Platform/compat ${ROBOTARM_HAL_INCLUDE_DIRS})

add_library(arm_bsp STATIC
    ${ROBOTARM_SOURCE_ROOT}/BSP/src/bsp_limit_trace.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/src/bsp_i2c_recovery.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/src/bsp_limit_irq.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/src/bsp_robot_board.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/src/uart_line_reader.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/src/bsp_led.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/src/bsp_uart1.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/driver/src/drv_as5600.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/driver/src/drv_ps2.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/driver/src/ps2_bus.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/driver/src/drv_servo.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/driver/src/drv_stepper.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/driver/src/drv_tmc2209.c
)
robotarm_target(arm_bsp)
target_include_directories(arm_bsp PUBLIC
    ${ROBOTARM_SOURCE_ROOT}/BSP/include
    ${ROBOTARM_SOURCE_ROOT}/BSP/driver/include)
target_include_directories(arm_bsp PRIVATE
    Core/Inc ${ROBOTARM_SOURCE_ROOT}/Platform ${ROBOTARM_SOURCE_ROOT}/Algorithm/include ${ROBOTARM_SOURCE_ROOT}/Algorithm/include/common ${ROBOTARM_HAL_INCLUDE_DIRS})
target_link_libraries(arm_bsp PRIVATE arm_platform)

add_library(arm_device STATIC
    ${ROBOTARM_SOURCE_ROOT}/Device/src/dev_gripper.c
    ${ROBOTARM_SOURCE_ROOT}/Device/src/dev_input.c
    ${ROBOTARM_SOURCE_ROOT}/Device/src/dev_joint.c
    ${ROBOTARM_SOURCE_ROOT}/Device/src/dev_limit_switch.c
)
robotarm_target(arm_device)
target_include_directories(arm_device PUBLIC ${ROBOTARM_SOURCE_ROOT}/Device/include)
target_include_directories(arm_device PRIVATE
    Core/Inc ${ROBOTARM_SOURCE_ROOT}/BSP/include ${ROBOTARM_SOURCE_ROOT}/Algorithm/include ${ROBOTARM_SOURCE_ROOT}/Algorithm/include/common ${ROBOTARM_HAL_INCLUDE_DIRS})
target_link_libraries(arm_device PUBLIC arm_bsp)

# Algorithm contains pure robot mathematics and kinematics.  It is kept free
# of HAL, BSP, Device, Service and App dependencies so it remains host-testable.
add_library(arm_algorithm STATIC
    ${ROBOTARM_SOURCE_ROOT}/Algorithm/common/limit_state_filter.c
    ${ROBOTARM_SOURCE_ROOT}/Algorithm/common/home_sequence.c
    ${ROBOTARM_SOURCE_ROOT}/Algorithm/common/robot_math.c
    ${ROBOTARM_SOURCE_ROOT}/Algorithm/common/robot_home_pose.c
    ${ROBOTARM_SOURCE_ROOT}/Algorithm/kinematics/kinematics.c
)
robotarm_target(arm_algorithm)
target_include_directories(arm_algorithm PUBLIC ${ROBOTARM_SOURCE_ROOT}/Algorithm/include ${ROBOTARM_SOURCE_ROOT}/Algorithm/include/common)

add_library(arm_service STATIC
    ${ROBOTARM_SOURCE_ROOT}/Service/homing/svc_homing_sequence.c
    ${ROBOTARM_SOURCE_ROOT}/Service/safety/limit_irq_guard.c
    ${ROBOTARM_SOURCE_ROOT}/Service/diagnostics/limit_observation_service.c
    ${ROBOTARM_SOURCE_ROOT}/Service/waveform/waveform_service.c
    ${ROBOTARM_SOURCE_ROOT}/Service/command/command_service.c
    ${ROBOTARM_SOURCE_ROOT}/Service/planning/planning_service.c
    ${ROBOTARM_SOURCE_ROOT}/Service/motion/motion_service.c
    ${ROBOTARM_SOURCE_ROOT}/Service/state/state_service.c
    ${ROBOTARM_SOURCE_ROOT}/Service/safety/safety_service.c
    ${ROBOTARM_SOURCE_ROOT}/Service/gripper/svc_gripper.c
    ${ROBOTARM_SOURCE_ROOT}/Service/homing/svc_homing.c
    ${ROBOTARM_SOURCE_ROOT}/Service/motion/internal/ctrl_motion_engine.c
    ${ROBOTARM_SOURCE_ROOT}/Service/planning/internal/ctrl_planner.c
    ${ROBOTARM_SOURCE_ROOT}/Service/motion/internal/ctrl_closed_loop.c
)
robotarm_target(arm_service)
target_include_directories(arm_service PUBLIC ${ROBOTARM_SOURCE_ROOT}/Service/include)
target_include_directories(arm_service PRIVATE
    Core/Inc
    ${ROBOTARM_SOURCE_ROOT}/Service/motion/internal
    ${ROBOTARM_SOURCE_ROOT}/Service/planning/internal
    ${ROBOTARM_SOURCE_ROOT}/BSP/include ${ROBOTARM_SOURCE_ROOT}/Device/include
    ${ROBOTARM_SOURCE_ROOT}/Algorithm/include ${ROBOTARM_SOURCE_ROOT}/Algorithm/include/common
    ${ROBOTARM_SOURCE_ROOT}/Platform ${ROBOTARM_SOURCE_ROOT}/Platform/compat
    ${ROBOTARM_HAL_INCLUDE_DIRS})
target_link_libraries(arm_service PRIVATE arm_algorithm arm_device arm_platform)

add_library(arm_app STATIC
    ${ROBOTARM_SOURCE_ROOT}/App/app_waveform.c
    ${ROBOTARM_SOURCE_ROOT}/App/app_serial_protocol.c
    ${ROBOTARM_SOURCE_ROOT}/App/app_diagnostics.c
    ${ROBOTARM_SOURCE_ROOT}/App/app_runtime.c
    ${ROBOTARM_SOURCE_ROOT}/App/app_gcode_parser.c
    ${ROBOTARM_SOURCE_ROOT}/App/app_gcode_exec.c
    ${ROBOTARM_SOURCE_ROOT}/App/app_teleop.c
    ${ROBOTARM_SOURCE_ROOT}/App/app_calibration.c
)
robotarm_target(arm_app)
target_include_directories(arm_app PUBLIC ${ROBOTARM_SOURCE_ROOT}/App/include)
target_include_directories(arm_app PRIVATE
    ${ROBOTARM_SOURCE_ROOT}/App/adapters
    ${ROBOTARM_SOURCE_ROOT}/Service/include
    ${ROBOTARM_SOURCE_ROOT}/Algorithm/include ${ROBOTARM_SOURCE_ROOT}/Algorithm/include/common
    ${ROBOTARM_SOURCE_ROOT}/Platform ${ROBOTARM_SOURCE_ROOT}/Platform/compat)
target_link_libraries(arm_app PRIVATE arm_service arm_platform arm_algorithm)

# Adapters are the only App code allowed to know STM32 handles, BSP or Device.
add_library(arm_app_adapters STATIC
    ${ROBOTARM_SOURCE_ROOT}/App/adapters/app_hardware_adapter.c
    ${ROBOTARM_SOURCE_ROOT}/App/adapters/app_isr_adapter.c
)
robotarm_target(arm_app_adapters)
target_include_directories(arm_app_adapters PRIVATE
    Core/Inc ${ROBOTARM_SOURCE_ROOT}/App/include ${ROBOTARM_SOURCE_ROOT}/App/adapters
    ${ROBOTARM_SOURCE_ROOT}/Service/include ${ROBOTARM_SOURCE_ROOT}/BSP/include ${ROBOTARM_SOURCE_ROOT}/Device/include
    ${ROBOTARM_SOURCE_ROOT}/Algorithm/include ${ROBOTARM_SOURCE_ROOT}/Algorithm/include/common
    ${ROBOTARM_SOURCE_ROOT}/Platform ${ROBOTARM_HAL_INCLUDE_DIRS})
target_link_libraries(arm_app_adapters PRIVATE arm_service arm_device arm_bsp)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc ${ROBOTARM_SOURCE_ROOT}/App/include ${ROBOTARM_SOURCE_ROOT}/App/adapters
    ${ROBOTARM_SOURCE_ROOT}/Platform)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    arm_app
    arm_app_adapters
    arm_service
    arm_algorithm
    arm_device
    arm_bsp
    arm_platform
    stm32cubemx)

find_package(Python3 COMPONENTS Interpreter QUIET)
if(ROBOTARM_STARTUP_TRIAL_MODE)
    if(ROBOTARM_LIMIT_SEEK_TRIAL)
        target_sources(${CMAKE_PROJECT_NAME} PRIVATE tools/limit_seek_trial/guard.c tools/limit_seek_trial/seek_logic.c tools/limit_seek_trial/contact_gate.c tools/limit_seek_trial/seek_result.c tools/limit_seek_trial/boot_contact.c tools/limit_seek_trial/input_hold.c)
        target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE tools/startup_trial tools/limit_seek_trial)
        if(LIMIT_SEEK_UNTIL_CONTACT)
            target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE TRIAL_DEADLINE_MS=95000U SEEK_MAX_STEPS=1500U SEEK_AXIS_TIMEOUT_MS=30000U SEEK_ALLOW_PRESSED_START=1)
        else()
            target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE TRIAL_DEADLINE_MS=15000U)
        endif()
        set_source_files_properties(tools/limit_seek_trial/guard.c tools/limit_seek_trial/seek_logic.c tools/limit_seek_trial/contact_gate.c tools/limit_seek_trial/seek_result.c tools/limit_seek_trial/boot_contact.c tools/limit_seek_trial/input_hold.c PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror")
    else()
        target_sources(${CMAKE_PROJECT_NAME} PRIVATE tools/startup_trial/guard.c)
    endif()
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE tools/startup_trial/trial_logic.c tools/startup_trial/ready_hold.c)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE STARTUP_TRIAL_READY_HOLD_MS=${STARTUP_TRIAL_READY_HOLD_MS})
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE RobotArm/BSP/driver/include RobotArm/Device/include RobotArm/Service/include RobotArm/Algorithm/include/common)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ROBOTARM_STARTUP_TRIAL_MODE=1 ROBOTARM_IRQ_EVIDENCE_MODE=$<BOOL:${ROBOTARM_IRQ_EVIDENCE_MODE}> STARTUP_TRIAL_MOTION_ENABLED=$<BOOL:${STARTUP_TRIAL_MOTION_ENABLED}> HOMING_STEP_DELAY_US=20000 HOMING_MAX_STEPS=200)
    if(ROBOTARM_SEQUENTIAL_HOMING)
        target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE TRIAL_DEADLINE_MS=220000U)
    endif()
    set_source_files_properties(tools/startup_trial/guard.c tools/startup_trial/trial_logic.c tools/startup_trial/ready_hold.c PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror")
    foreach(symbol IN ITEMS main HAL_IncTick HAL_GPIO_WritePin HAL_TIM_PWM_Start Dev_Joint_Step AppHardware_ReadLine AppSerial_SetMode AppHardware_StartStepTimer)
        target_link_options(${CMAKE_PROJECT_NAME} PRIVATE "-Wl,--wrap=${symbol}")
    endforeach()
endif()
if(ROBOTARM_STARTUP_AUDIT_MODE)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE tools/startup_audit/guard.c)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE RobotArm/BSP/driver/include)
    target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,--wrap=main
        -Wl,--wrap=HAL_IncTick -Wl,--wrap=HAL_GPIO_WritePin
        -Wl,--wrap=HAL_TIM_PWM_Start -Wl,--wrap=HAL_TIM_Base_Start_IT
        -Wl,--wrap=Drv_Stepper_Step -Wl,--wrap=AppHardware_PrepareMotionDrivers
        -Wl,--wrap=AppHardware_ReadMotorAngle)
endif()
if(ROBOTARM_IRQ_EVIDENCE_MODE)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE tools/startup_audit/irq_evidence.c)
    target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,--wrap=App_ISR_OnLimitVectorEnter
        -Wl,--wrap=App_ISR_OnLimitVectorExit -Wl,--wrap=HAL_GPIO_EXTI_IRQHandler -Wl,--wrap=App_ISR_OnGpioEdge)
    set_source_files_properties(tools/startup_audit/irq_evidence.c PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror")
endif()
if(Python3_Interpreter_FOUND)
    add_custom_target(check_architecture_dependencies
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/check_architecture_dependencies.py
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Checking RobotArm architecture dependencies"
    )
endif()
