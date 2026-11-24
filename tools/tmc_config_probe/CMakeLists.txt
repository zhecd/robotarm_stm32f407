# 仅链接真实TMC/BSP配置链；没有App、回零、夹爪、STEP时钟或电机测试入口。
add_executable(${CMAKE_PROJECT_NAME} main.c
    ${CMAKE_SOURCE_DIR}/startup_stm32f407xx.s
    ${CMAKE_SOURCE_DIR}/Core/Src/system_stm32f4xx.c
    ${CMAKE_SOURCE_DIR}/Core/Src/usart.c
    ${CMAKE_SOURCE_DIR}/RobotArm/BSP/src/bsp_robot_board.c
    ${CMAKE_SOURCE_DIR}/RobotArm/BSP/driver/src/drv_stepper.c
    ${CMAKE_SOURCE_DIR}/RobotArm/BSP/driver/src/drv_tmc2209.c)
foreach(module IN ITEMS hal hal_gpio hal_uart hal_rcc hal_rcc_ex hal_cortex hal_pwr hal_dma)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_${module}.c)
endforeach()
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE STM32F407xx USE_HAL_DRIVER)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/RobotArm/BSP/include
    ${CMAKE_SOURCE_DIR}/RobotArm/BSP/driver/include
    ${CMAKE_SOURCE_DIR}/RobotArm/Algorithm/include/common
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Werror)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,--wrap=HAL_GPIO_WritePin)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407VETx_FLASH.ld\" --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,-Map=tmc_config_probe.map -Wl,--print-memory-usage")
