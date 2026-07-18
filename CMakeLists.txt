cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME robotarm_stm32f407)
include("cmake/gcc-arm-none-eabi.cmake")
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME} C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

# All hand-written robot firmware lives below this directory.  CubeMX output
# intentionally remains at the project root in Core/ and Drivers/.
set(ROBOTARM_SOURCE_ROOT RobotArm)

set(ROBOTARM_HAL_INCLUDE_DIRS
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
)

function(robotarm_target target)
    target_compile_definitions(${target} PRIVATE USE_HAL_DRIVER STM32F407xx)
endfunction()

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
    ${ROBOTARM_SOURCE_ROOT}/BSP/src/bsp_led.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/src/bsp_uart1.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/driver/src/drv_as5600.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/driver/src/drv_ps2.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/driver/src/drv_servo.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/driver/src/drv_stepper.c
    ${ROBOTARM_SOURCE_ROOT}/BSP/driver/src/drv_tmc2209.c
)
robotarm_target(arm_bsp)
target_include_directories(arm_bsp PUBLIC
    ${ROBOTARM_SOURCE_ROOT}/BSP/include
    ${ROBOTARM_SOURCE_ROOT}/BSP/driver/include)
target_include_directories(arm_bsp PRIVATE
    Core/Inc ${ROBOTARM_SOURCE_ROOT}/Domain/include ${ROBOTARM_SOURCE_ROOT}/Domain/include/common ${ROBOTARM_HAL_INCLUDE_DIRS})

add_library(arm_device STATIC
    ${ROBOTARM_SOURCE_ROOT}/Device/src/dev_gripper.c
    ${ROBOTARM_SOURCE_ROOT}/Device/src/dev_input.c
    ${ROBOTARM_SOURCE_ROOT}/Device/src/dev_joint.c
    ${ROBOTARM_SOURCE_ROOT}/Device/src/dev_limit_switch.c
)
robotarm_target(arm_device)
target_include_directories(arm_device PUBLIC ${ROBOTARM_SOURCE_ROOT}/Device/include)
target_include_directories(arm_device PRIVATE
    Core/Inc ${ROBOTARM_SOURCE_ROOT}/BSP/include ${ROBOTARM_SOURCE_ROOT}/Domain/include ${ROBOTARM_SOURCE_ROOT}/Domain/include/common ${ROBOTARM_HAL_INCLUDE_DIRS})
target_link_libraries(arm_device PUBLIC arm_bsp)

# Domain contains pure robot mathematics and kinematics.  It is kept free of
# HAL, BSP, Device, Service and App dependencies so it remains host-testable.
add_library(arm_domain STATIC
    ${ROBOTARM_SOURCE_ROOT}/Domain/common/robot_math.c
    ${ROBOTARM_SOURCE_ROOT}/Domain/common/robot_home_pose.c
    ${ROBOTARM_SOURCE_ROOT}/Domain/kinematics/kinematics.c
)
robotarm_target(arm_domain)
target_include_directories(arm_domain PUBLIC ${ROBOTARM_SOURCE_ROOT}/Domain/include ${ROBOTARM_SOURCE_ROOT}/Domain/include/common)

add_library(arm_service STATIC
    ${ROBOTARM_SOURCE_ROOT}/Service/command/command_service.c
    ${ROBOTARM_SOURCE_ROOT}/Service/motion/motion_service.c
    ${ROBOTARM_SOURCE_ROOT}/Service/state/state_service.c
    ${ROBOTARM_SOURCE_ROOT}/Service/safety/safety_service.c
    ${ROBOTARM_SOURCE_ROOT}/Service/gripper/svc_gripper.c
    ${ROBOTARM_SOURCE_ROOT}/Service/homing/svc_homing.c
    ${ROBOTARM_SOURCE_ROOT}/Service/motion/internal/ctrl_motion_engine.c
    ${ROBOTARM_SOURCE_ROOT}/Service/motion/internal/ctrl_planner.c
    ${ROBOTARM_SOURCE_ROOT}/Service/motion/internal/ctrl_closed_loop.c
)
robotarm_target(arm_service)
target_include_directories(arm_service PUBLIC ${ROBOTARM_SOURCE_ROOT}/Service/include)
target_include_directories(arm_service PRIVATE
    Core/Inc
    ${ROBOTARM_SOURCE_ROOT}/Service/motion/internal
    ${ROBOTARM_SOURCE_ROOT}/BSP/include ${ROBOTARM_SOURCE_ROOT}/Device/include
    ${ROBOTARM_SOURCE_ROOT}/Domain/include ${ROBOTARM_SOURCE_ROOT}/Domain/include/common
    ${ROBOTARM_SOURCE_ROOT}/Platform ${ROBOTARM_SOURCE_ROOT}/Platform/compat
    ${ROBOTARM_HAL_INCLUDE_DIRS})
target_link_libraries(arm_service PRIVATE arm_domain arm_device arm_platform)

add_library(arm_app STATIC
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
    ${ROBOTARM_SOURCE_ROOT}/Domain/include ${ROBOTARM_SOURCE_ROOT}/Domain/include/common
    ${ROBOTARM_SOURCE_ROOT}/Platform ${ROBOTARM_SOURCE_ROOT}/Platform/compat)
target_link_libraries(arm_app PRIVATE arm_service arm_platform arm_domain)

# Adapters are the only App code allowed to know STM32 handles, BSP or Device.
add_library(arm_app_adapters STATIC
    ${ROBOTARM_SOURCE_ROOT}/App/adapters/app_hardware_adapter.c
    ${ROBOTARM_SOURCE_ROOT}/App/adapters/app_isr_adapter.c
)
robotarm_target(arm_app_adapters)
target_include_directories(arm_app_adapters PRIVATE
    Core/Inc ${ROBOTARM_SOURCE_ROOT}/App/include ${ROBOTARM_SOURCE_ROOT}/App/adapters
    ${ROBOTARM_SOURCE_ROOT}/Service/include ${ROBOTARM_SOURCE_ROOT}/BSP/include ${ROBOTARM_SOURCE_ROOT}/Device/include
    ${ROBOTARM_SOURCE_ROOT}/Domain/include ${ROBOTARM_SOURCE_ROOT}/Domain/include/common
    ${ROBOTARM_HAL_INCLUDE_DIRS})
target_link_libraries(arm_app_adapters PRIVATE arm_service arm_device arm_bsp)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc ${ROBOTARM_SOURCE_ROOT}/App/include ${ROBOTARM_SOURCE_ROOT}/App/adapters)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    arm_app
    arm_app_adapters
    arm_service
    arm_domain
    arm_device
    arm_bsp
    arm_platform
    stm32cubemx)

find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    add_custom_target(check_architecture_dependencies
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/check_architecture_dependencies.py
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Checking RobotArm architecture dependencies"
    )
endif()
