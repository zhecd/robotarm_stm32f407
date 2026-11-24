# 独立夹爪入口，不链接App、关节设备、TMC、步进驱动、TIM6或自动回零。
add_executable(${CMAKE_PROJECT_NAME} main.c gripper_logic.c
    ../m1_homing/m1_rx.c ../m1_homing/m1_limits.c ../m1_homing/m1_deadline.c
    ${CMAKE_SOURCE_DIR}/startup_stm32f407xx.s
    ${CMAKE_SOURCE_DIR}/Core/Src/system_stm32f4xx.c
    ${CMAKE_SOURCE_DIR}/Core/Src/sysmem.c
    ${CMAKE_SOURCE_DIR}/Core/Src/usart.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32f4xx_hal_msp.c
    ${CMAKE_SOURCE_DIR}/RobotArm/BSP/driver/src/drv_servo.c)
foreach(module IN ITEMS hal hal_gpio hal_uart hal_rcc hal_rcc_ex hal_cortex hal_pwr hal_dma hal_tim hal_tim_ex)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_${module}.c)
endforeach()
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE STM32F407xx USE_HAL_DRIVER
    GRIPPER_TEST_MOTION_ENABLED=$<BOOL:${GRIPPER_TEST_MOTION_ENABLED}>)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ../m1_homing
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/RobotArm/BSP/driver/include
    ${CMAKE_SOURCE_DIR}/RobotArm/Algorithm/include/common
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Werror)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407VETx_FLASH.ld\" --specs=nano.specs --specs=nosys.specs -Wl,-u,_printf_float -Wl,--gc-sections -Wl,-Map=gripper_test.map -Wl,--print-memory-usage -lm")
