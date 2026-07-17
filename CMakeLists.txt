cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME robotarm_stm32f407)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/main.c
    Core/Src/gpio.c

    # Board support
    Core/Src/bsp/bsp_led.c
    Core/Src/bsp/bsp_uart1.c

    # Device drivers
    Core/Src/driver/drv_as5600.c
    Core/Src/driver/drv_ps2.c
    Core/Src/driver/drv_servo.c
    Core/Src/driver/drv_stepper.c
    Core/Src/driver/drv_tmc2209.c

    # Robot devices
    Core/Src/device/dev_gripper.c
    Core/Src/device/dev_input.c
    Core/Src/device/dev_joint.c
    Core/Src/device/dev_limit_switch.c

    # Robot services and control algorithms
    Core/Src/service/svc_gripper.c
    Core/Src/service/svc_homing.c
    Core/Src/service/control/ctrl_motion_engine.c
    Core/Src/service/control/ctrl_kinematics.c
    Core/Src/service/control/ctrl_planner.c
    Core/Src/service/control/ctrl_closed_loop.c
    Core/Src/service/control/ctrl_compensation.c

    # App layer
    Core/Src/app/app_gcode_parser.c
    Core/Src/app/app_gcode_exec.c
    Core/Src/app/app_teleop.c
    Core/Src/app/app_calibration.c

    # Common
    Core/Src/common/robot_math.c
    Core/Src/common/robot_home_pose.c

    # Bare-metal OS abstraction
    Core/Src/os/os_adapter.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    Core/Inc/bsp
    Core/Inc/app
    Core/Inc/service
    Core/Inc/service/control
    Core/Inc/device
    Core/Inc/driver
    Core/Inc/common
    Core/Inc/os
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
