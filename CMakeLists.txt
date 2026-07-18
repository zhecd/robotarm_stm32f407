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

set(ROBOTARM_PROJECT_INCLUDE_DIRS
    Core/Inc
    App/include
    BSP/include
    Device/include
    Driver/include
    Domain/include
    Domain/include/common
    Service/include
    Platform
    Platform/compat
)

set(ROBOTARM_HAL_INCLUDE_DIRS
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
)

function(robotarm_layer target)
    target_include_directories(${target} PUBLIC ${ROBOTARM_PROJECT_INCLUDE_DIRS} ${ROBOTARM_HAL_INCLUDE_DIRS})
    target_compile_definitions(${target} PUBLIC USE_HAL_DRIVER STM32F407xx)
endfunction()

# Platform is the only location for bare-metal time, delay and critical-section
# primitives. os_adapter.c remains a temporary compatibility facade.
add_library(arm_platform STATIC
    Platform/platform_time.c
    Platform/platform_delay.c
    Platform/platform_critical.c
    Platform/compat/os/os_adapter.c
)
robotarm_layer(arm_platform)

add_library(arm_bsp STATIC
    BSP/src/bsp_led.c
    BSP/src/bsp_uart1.c
)
robotarm_layer(arm_bsp)

add_library(arm_driver STATIC
    Driver/src/drv_as5600.c
    Driver/src/drv_ps2.c
    Driver/src/drv_servo.c
    Driver/src/drv_stepper.c
    Driver/src/drv_tmc2209.c
)
robotarm_layer(arm_driver)
target_link_libraries(arm_driver PUBLIC arm_bsp)

add_library(arm_device STATIC
    Device/src/dev_gripper.c
    Device/src/dev_input.c
    Device/src/dev_joint.c
    Device/src/dev_limit_switch.c
)
robotarm_layer(arm_device)
target_link_libraries(arm_device PUBLIC arm_driver arm_bsp)

# Domain contains pure robot mathematics and kinematics.  It is kept free of
# HAL, BSP, Device, Service and App dependencies so it remains host-testable.
add_library(arm_domain STATIC
    Domain/common/robot_math.c
    Domain/common/robot_home_pose.c
    Domain/kinematics/kinematics.c
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
target_link_libraries(arm_service PUBLIC arm_domain arm_device arm_platform)

add_library(arm_app STATIC
    App/app_runtime.c
    App/app_gcode_parser.c
    App/app_gcode_exec.c
    App/app_teleop.c
    App/app_calibration.c
)
robotarm_layer(arm_app)
target_link_libraries(arm_app PUBLIC arm_service arm_device arm_platform arm_bsp)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${ROBOTARM_PROJECT_INCLUDE_DIRS})
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE arm_app stm32cubemx)

find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    add_custom_target(check_architecture_dependencies
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/check_architecture_dependencies.py
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Checking RobotArm architecture dependencies"
    )
endif()
