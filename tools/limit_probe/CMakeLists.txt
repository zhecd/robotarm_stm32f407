# 独立诊断只链接ST启动/CMSIS和本目录，不链接HAL及RobotArm业务库。
option(LIMIT_PROBE_UART "Add polled USART1 and pure query formatter to isolated probe" OFF)
add_executable(${CMAKE_PROJECT_NAME}
    ${CMAKE_SOURCE_DIR}/startup_stm32f407xx.s
    ${CMAKE_SOURCE_DIR}/Core/Src/system_stm32f4xx.c
    main.c probe_logic.c)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE STM32F407xx)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE LIMIT_PROBE_UART=$<BOOL:${LIMIT_PROBE_UART}>)
if(LIMIT_PROBE_UART)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE probe_console.c
        ${CMAKE_SOURCE_DIR}/RobotArm/BSP/src/uart_line_reader.c)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/RobotArm/BSP/include)
endif()
# 不强拉浮点printf；默认入口无串口，可选UART仅用整数/文本snprintf。
set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407VETx_FLASH.ld\" --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,-Map=limit_probe.map -Wl,--print-memory-usage" )
