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

set(ROBOTARM_LEGACY_INCLUDE_DIRS
    Core/Inc
    Core/Inc/app
    Core/Inc/bsp
    Core/Inc/common
    Core/Inc/device
    Core/Inc/driver
    Core/Inc/os
    Core/Inc/service
    Core/Inc/service/control
)

set(ROBOTARM_HAL_INCLUDE_DIRS
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
)

function(robotarm_layer target)
    target_include_directories(${target} PUBLIC ${ROBOTARM_LEGACY_INCLUDE_DIRS} ${ROBOTARM_HAL_INCLUDE_DIRS})
    target_compile_definitions(${target} PUBLIC USE_HAL_DRIVER STM32F407xx)
endfunction()

# Platform is the only location for bare-metal time, delay and critical-section
# primitives. os_adapter.c remains a temporary compatibility facade.
add_library(arm_platform STATIC
    Platform/platform_time.c
    Platform/platform_delay.c
    Platform/platform_critical.c
    Core/Src/os/os_adapter.c
)
robotarm_layer(arm_platform)
target_include_directories(arm_platform PUBLIC Platform)

add_library(arm_bsp STATIC
    Core/Src/bsp/bsp_led.c
    Core/Src/bsp/bsp_uart1.c
)
robotarm_layer(arm_bsp)

add_library(arm_driver STATIC
    Core/Src/driver/drv_as5600.c
    Core/Src/driver/drv_ps2.c
    Core/Src/driver/drv_servo.c
    Core/Src/driver/drv_stepper.c
    Core/Src/driver/drv_tmc2209.c
)
robotarm_layer(arm_driver)
target_link_libraries(arm_driver PUBLIC arm_bsp)

add_library(arm_device STATIC
    Core/Src/device/dev_gripper.c
    Core/Src/device/dev_input.c
    Core/Src/device/dev_joint.c
    Core/Src/device/dev_limit_switch.c
)
robotarm_layer(arm_device)
target_link_libraries(arm_device PUBLIC arm_driver arm_bsp)

# The first migration pass keeps pure math and existing kinematics together.
# Trajectory, interpolation and PID are promoted here in the later Domain pass.
add_library(arm_domain STATIC
    Domain/common/robot_math.c
    Domain/common/robot_home_pose.c
    Domain/kinematics/ctrl_kinematics.c
)
robotarm_layer(arm_domain)
target_link_libraries(arm_domain PUBLIC arm_platform)

add_library(arm_service STATIC
    Service/command/command_service.c
    Service/motion/motion_service.c
    Service/state/state_service.c
    Service/safety/safety_service.c
    Service/gripper/svc_gripper.c
    Service/homing/svc_homing.c
    Service/control/ctrl_motion_engine.c
    Service/control/ctrl_planner.c
    Service/control/ctrl_closed_loop.c
    Service/control/ctrl_compensation.c
)
robotarm_layer(arm_service)
target_include_directories(arm_service PUBLIC Service/command Service/motion Service/state Service/safety)
target_link_libraries(arm_service PUBLIC arm_domain arm_device arm_platform)

add_library(arm_app STATIC
    App/app_runtime.c
    App/app_gcode_parser.c
    App/app_gcode_exec.c
    App/app_teleop.c
    App/app_calibration.c
)
robotarm_layer(arm_app)
target_include_directories(arm_app PUBLIC App)
target_link_libraries(arm_app PUBLIC arm_service arm_device arm_platform arm_bsp)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE App Platform ${ROBOTARM_LEGACY_INCLUDE_DIRS})
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE arm_app stm32cubemx)

find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    add_custom_target(check_architecture_dependencies
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/check_architecture_dependencies.py
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Checking RobotArm architecture dependencies"
    )
endif()
