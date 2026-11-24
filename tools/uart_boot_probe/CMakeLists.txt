# 物理返回链路对照：不链接App、运动/夹爪/PS2/TMC、HAL UART或发送环。
add_executable(${CMAKE_PROJECT_NAME}
    ${CMAKE_SOURCE_DIR}/startup_stm32f407xx.s
    ${CMAKE_SOURCE_DIR}/Core/Src/system_stm32f4xx.c
    main.c)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE STM32F407xx)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Werror)
set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407VETx_FLASH.ld\" --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,-Map=uart_boot_probe.map -Wl,--print-memory-usage")
