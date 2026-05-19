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
set(CMAKE_PROJECT_NAME F407AS5600_2)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
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
    # ── CubeMX generated (root level) ──
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/i2c.c
    Core/Src/tim.c
    Core/Src/usart.c

    # ── Common layer ──
    Core/Src/common/common.c

    # ── BSP layer ──
    Core/Src/bsp/bsp_led.c
    Core/Src/bsp/bsp_stepper.c
    Core/Src/bsp/bsp_tmc2209.c
    Core/Src/bsp/bsp_uart1.c
    Core/Src/bsp/bsp_ps2.c
    Core/Src/bsp/bsp_gripper.c
    Core/Src/bsp/bsp_as5600.c
    Core/Src/bsp/bsp_homing.c

    # ── Control layer ──
    Core/Src/control/ctrl_motion_engine.c
    Core/Src/control/ctrl_kinematics.c
    Core/Src/control/ctrl_planner.c
    Core/Src/control/ctrl_closed_loop.c
    Core/Src/control/ctrl_compensation.c

    # ── Application layer ──
    Core/Src/app/app_teleop.c
    Core/Src/app/app_gcode_parser.c
    Core/Src/app/app_gcode_exec.c
    Core/Src/app/app_calibration.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    Core/Inc/common
    Core/Inc/bsp
    Core/Inc/control
    Core/Inc/app
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
