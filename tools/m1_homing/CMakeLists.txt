# 独立M1测试，不链接App、三轴回零、夹爪、TIM2或普通运动规划。
add_executable(${CMAKE_PROJECT_NAME} main.c m1_logic.c m1_rx.c m1_deadline.c m1_limits.c m1_reference.c axis_binding.c
    ${CMAKE_SOURCE_DIR}/startup_stm32f407xx.s
    ${CMAKE_SOURCE_DIR}/Core/Src/system_stm32f4xx.c
    ${CMAKE_SOURCE_DIR}/Core/Src/sysmem.c
    ${CMAKE_SOURCE_DIR}/Core/Src/usart.c
    ${CMAKE_SOURCE_DIR}/Core/Src/i2c.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32f4xx_hal_msp.c
    ${CMAKE_SOURCE_DIR}/RobotArm/Device/src/dev_joint.c
    ${CMAKE_SOURCE_DIR}/RobotArm/BSP/src/bsp_robot_board.c
    ${CMAKE_SOURCE_DIR}/RobotArm/BSP/src/bsp_i2c_recovery.c
    ${CMAKE_SOURCE_DIR}/RobotArm/BSP/driver/src/drv_stepper.c
    ${CMAKE_SOURCE_DIR}/RobotArm/BSP/driver/src/drv_tmc2209.c
    ${CMAKE_SOURCE_DIR}/RobotArm/BSP/driver/src/drv_as5600.c
    ${CMAKE_SOURCE_DIR}/RobotArm/Algorithm/common/robot_math.c
    ${CMAKE_SOURCE_DIR}/RobotArm/Algorithm/common/robot_home_pose.c
    ${CMAKE_SOURCE_DIR}/RobotArm/Platform/platform_time.c
    ${CMAKE_SOURCE_DIR}/RobotArm/Platform/platform_delay.c
    ${CMAKE_SOURCE_DIR}/RobotArm/Platform/platform_critical.c)
foreach(module IN ITEMS hal hal_gpio hal_uart hal_rcc hal_rcc_ex hal_cortex hal_pwr hal_dma hal_i2c hal_i2c_ex)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_${module}.c)
endforeach()
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE STM32F407xx USE_HAL_DRIVER
    SINGLE_AXIS_ID=${SINGLE_AXIS_ID}
    M1_HOME_MOTION_ENABLED=$<BOOL:${M1_HOME_MOTION_ENABLED}>
    M1_JOG_MOTOR_TEN_DEG=$<BOOL:${M1_JOG_MOTOR_TEN_DEG}>
    M1_JOG_JOINT_TEN_DEG=$<BOOL:${M1_JOG_JOINT_TEN_DEG}>)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/RobotArm/Device/include
    ${CMAKE_SOURCE_DIR}/RobotArm/BSP/include
    ${CMAKE_SOURCE_DIR}/RobotArm/BSP/driver/include
    ${CMAKE_SOURCE_DIR}/RobotArm/Algorithm/include
    ${CMAKE_SOURCE_DIR}/RobotArm/Algorithm/include/common
    ${CMAKE_SOURCE_DIR}/RobotArm/Platform
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Werror)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407VETx_FLASH.ld\" --specs=nano.specs --specs=nosys.specs -Wl,-u,_printf_float -Wl,--gc-sections -Wl,-Map=m1_homing.map -Wl,--print-memory-usage -lm")
