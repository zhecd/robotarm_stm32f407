# 独立Flash入口：不链接自动回零、夹爪、STEP定时器或业务栈。
option(TMC_BENCH_LEGACY "Use byte-identical original ZIP drivers and real ST HAL" OFF)
add_executable(${CMAKE_PROJECT_NAME} main.c bench_logic.c
    ${CMAKE_SOURCE_DIR}/startup_stm32f407xx.s
    ${CMAKE_SOURCE_DIR}/Core/Src/system_stm32f4xx.c)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE STM32F407xx)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE TMC_BENCH_LEGACY=$<BOOL:${TMC_BENCH_LEGACY}>)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include)
if(TMC_BENCH_LEGACY)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE legacy_bridge.c
        legacy_original/src/drv_stepper.c legacy_original/src/drv_tmc2209.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE legacy_original/include
        ${CMAKE_SOURCE_DIR}/Core/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USE_HAL_DRIVER)
    target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -Wl,--wrap=HAL_UART_Transmit -Wl,--wrap=HAL_GPIO_WritePin)
endif()
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Werror)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407VETx_FLASH.ld\" --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,-Map=tmc_bench.map -Wl,--print-memory-usage")
