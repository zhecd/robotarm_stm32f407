cmake_minimum_required(VERSION 3.20)

project(robotarm_logic_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
# 必须放在所有add_executable之前，较早声明的测试也要继承同样的检查策略。
# Release保留assert；新模块和既有测试均把编译警告当作错误。
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -UNDEBUG)
endif()
enable_testing()
add_executable(limit_seek_logic_tests limit_seek_logic_tests.c ../tools/limit_seek_trial/seek_logic.c)
target_include_directories(limit_seek_logic_tests PRIVATE ../tools/limit_seek_trial)
add_test(NAME limit_seek_logic_tests COMMAND limit_seek_logic_tests)
add_executable(limit_seek_extended_tests limit_seek_extended_tests.c ../tools/limit_seek_trial/seek_logic.c)
target_include_directories(limit_seek_extended_tests PRIVATE ../tools/limit_seek_trial)
target_compile_definitions(limit_seek_extended_tests PRIVATE SEEK_MAX_STEPS=1500U SEEK_AXIS_TIMEOUT_MS=30000U SEEK_ALLOW_PRESSED_START=1)
add_test(NAME limit_seek_extended_tests COMMAND limit_seek_extended_tests)
add_executable(irq_evidence_tests irq_evidence_tests.c ../tools/startup_audit/irq_evidence.c)
target_include_directories(irq_evidence_tests PRIVATE hal_stubs/limit_trace ../tools/startup_audit)
add_test(NAME irq_evidence_tests COMMAND irq_evidence_tests)
set_tests_properties(irq_evidence_tests PROPERTIES LABELS "stage5;irq_guard;evidence" TIMEOUT 10)
add_executable(irq_axis_evidence_tests irq_axis_evidence_tests.c ../tools/startup_audit/irq_evidence.c)
target_include_directories(irq_axis_evidence_tests PRIVATE hal_stubs/limit_trace ../tools/startup_audit)
foreach(scenario IN ITEMS fairness ring freeze shared publish transparent saturation)
    add_test(NAME irq_axis_evidence_${scenario} COMMAND irq_axis_evidence_tests ${scenario})
    set_tests_properties(irq_axis_evidence_${scenario} PROPERTIES LABELS "stage5;irq_guard;evidence" TIMEOUT 10)
endforeach()
add_executable(startup_trial_logic_tests startup_trial_logic_tests.c ../tools/startup_trial/trial_logic.c)
target_include_directories(startup_trial_logic_tests PRIVATE ../tools/startup_trial)
add_test(NAME startup_trial_logic_tests COMMAND startup_trial_logic_tests)
set_tests_properties(startup_trial_logic_tests PROPERTIES LABELS "stage3;startup;bounded" TIMEOUT 10)
add_executable(gripper_logic_tests gripper_logic_tests.c ../tools/gripper_test/gripper_logic.c)
target_include_directories(gripper_logic_tests PRIVATE ../tools/gripper_test)
add_test(NAME gripper_logic_tests COMMAND gripper_logic_tests)
set_tests_properties(gripper_logic_tests PROPERTIES LABELS "stage3;gripper;bounded" TIMEOUT 10)
add_executable(servo_driver_tests servo_driver_tests.c ../RobotArm/BSP/driver/src/drv_servo.c)
target_include_directories(servo_driver_tests PRIVATE hal_stubs/servo
    ../RobotArm/BSP/driver/include ../RobotArm/Algorithm/include/common)
add_test(NAME servo_driver_tests COMMAND servo_driver_tests)
set_tests_properties(servo_driver_tests PROPERTIES LABELS "stage3;driver;gripper" TIMEOUT 10)
foreach(scenario IN ITEMS rapid rapid_wrap rapid_permission rapid_stop_failure external_busy)
    add_test(NAME gripper_chain_${scenario} COMMAND gripper_result_chain_tests ${scenario})
    set_tests_properties(gripper_chain_${scenario} PROPERTIES LABELS "stage3;gripper;rapid" TIMEOUT 10)
endforeach()
add_executable(axis_binding_tests axis_binding_tests.c ../tools/m1_homing/axis_binding.c)
target_include_directories(axis_binding_tests PRIVATE ../tools/m1_homing)
foreach(axis RANGE 1 3)
    add_test(NAME axis_binding_${axis}_tests COMMAND axis_binding_tests ${axis})
    set_tests_properties(axis_binding_${axis}_tests PROPERTIES LABELS "stage3;axis;gpio" TIMEOUT 15)
endforeach()
add_executable(axis_limit_home_pipeline_tests axis_limit_home_pipeline_tests.c
    ../tools/m1_homing/axis_binding.c ../tools/m1_homing/m1_limits.c
    ../tools/m1_homing/m1_logic.c)
target_include_directories(axis_limit_home_pipeline_tests PRIVATE ../tools/m1_homing)
foreach(axis RANGE 1 3)
    add_test(NAME axis_limit_home_pipeline_${axis} COMMAND axis_limit_home_pipeline_tests ${axis})
    set_tests_properties(axis_limit_home_pipeline_${axis} PROPERTIES LABELS "stage3;axis;homing;limits;gpio" TIMEOUT 10)
endforeach()
add_executable(m1_reference_tests m1_reference_tests.c
    ../tools/m1_homing/m1_reference.c ../tools/m1_homing/m1_logic.c)
target_include_directories(m1_reference_tests PRIVATE ../tools/m1_homing)
target_link_libraries(m1_reference_tests PRIVATE m)
foreach(scenario IN ITEMS success gating timing faults numeric wrap replay home_failures)
    add_test(NAME m1_reference_${scenario}_tests COMMAND m1_reference_tests ${scenario})
    set_tests_properties(m1_reference_${scenario}_tests PROPERTIES LABELS "stage3;homing;reference" TIMEOUT 10)
endforeach()
add_executable(m1_limits_tests m1_limits_tests.c ../tools/m1_homing/m1_limits.c ../tools/m1_homing/m1_logic.c)
target_include_directories(m1_limits_tests PRIVATE ../tools/m1_homing)
add_test(NAME m1_limits_tests COMMAND m1_limits_tests)
set_tests_properties(m1_limits_tests PROPERTIES LABELS "stage3;homing;limits;led" TIMEOUT 10)
# 主机HOME监督脚本只用-SelfTest运行；该分支不创建串口，不接触硬件。
find_program(M1_HOME_POWERSHELL NAMES pwsh powershell)
if(M1_HOME_POWERSHELL)
    # 只用虚拟时钟/传输，覆盖READY丢弃RX后的查询重发，不连接板卡。
    add_test(NAME trial_query_transport
        COMMAND "${M1_HOME_POWERSHELL}" -NoProfile -NonInteractive -File
        "${CMAKE_CURRENT_LIST_DIR}/trial_query_transport_tests.ps1")
    set_tests_properties(trial_query_transport PROPERTIES LABELS "stage3;startup;host_contract" TIMEOUT 10)
    add_test(NAME startup_trial_host_contract COMMAND "${M1_HOME_POWERSHELL}" -NoProfile -NonInteractive -File
        "${CMAKE_SOURCE_DIR}/tools/startup_trial/execute_after_approval.ps1" -SelfTest)
    set_tests_properties(startup_trial_host_contract PROPERTIES LABELS "stage3;startup;host_contract" TIMEOUT 10)
    add_test(NAME gripper_host_contract
        COMMAND "${M1_HOME_POWERSHELL}" -NoProfile -NonInteractive -File
            "${CMAKE_SOURCE_DIR}/tools/gripper_test/execute_after_approval.ps1" -SelfTest)
    set_tests_properties(gripper_host_contract PROPERTIES LABELS "stage3;gripper;host_contract" TIMEOUT 10)
    add_test(NAME axis_home_host_contract
        COMMAND "${M1_HOME_POWERSHELL}" -NoProfile -NonInteractive -File
        "${CMAKE_CURRENT_LIST_DIR}/../tools/m1_homing/execute_axis_home_after_approval.ps1" -SelfTest)
    set_tests_properties(axis_home_host_contract PROPERTIES LABELS "stage3;axis;homing;host_contract" TIMEOUT 10)
    add_test(NAME axis_query_host_contract
        COMMAND "${M1_HOME_POWERSHELL}" -NoProfile -NonInteractive -File
        "${CMAKE_CURRENT_LIST_DIR}/../tools/m1_homing/query_axis_locked.ps1" -SelfTest)
    set_tests_properties(axis_query_host_contract PROPERTIES LABELS "stage3;axis;host_contract" TIMEOUT 10)
    add_test(NAME axis_jog_host_contract
        COMMAND "${M1_HOME_POWERSHELL}" -NoProfile -NonInteractive -File
        "${CMAKE_CURRENT_LIST_DIR}/../tools/m1_homing/execute_axis_jog_after_approval.ps1" -SelfTest)
    set_tests_properties(axis_jog_host_contract PROPERTIES LABELS "stage3;axis;host_contract" TIMEOUT 10)
    add_test(NAME m1_home_host_contract
        COMMAND "${M1_HOME_POWERSHELL}" -NoProfile -NonInteractive -File
        "${CMAKE_CURRENT_LIST_DIR}/../tools/m1_homing/execute_home_after_approval.ps1" -SelfTest)
    set_tests_properties(m1_home_host_contract PROPERTIES LABELS "stage3;homing;host_contract" TIMEOUT 10)
endif()
add_executable(m1_joint10_logic_tests m1_homing_logic_tests.c ../tools/m1_homing/m1_logic.c)
target_compile_definitions(m1_joint10_logic_tests PRIVATE M1_JOG_JOINT_TEN_DEG=1)
target_include_directories(m1_joint10_logic_tests PRIVATE ../tools/m1_homing)
add_test(NAME m1_joint10_logic_tests COMMAND m1_joint10_logic_tests)
set_tests_properties(m1_joint10_logic_tests PROPERTIES LABELS "stage3;homing;joint10" TIMEOUT 10)
add_executable(m1_joint10_deadline_tests m1_deadline_tests.c ../tools/m1_homing/m1_deadline.c)
target_compile_definitions(m1_joint10_deadline_tests PRIVATE M1_JOG_JOINT_TEN_DEG=1)
target_include_directories(m1_joint10_deadline_tests PRIVATE hal_stubs/m1_deadline ../tools/m1_homing)
add_test(NAME m1_joint10_deadline_tests COMMAND m1_joint10_deadline_tests)
set_tests_properties(m1_joint10_deadline_tests PROPERTIES LABELS "stage3;homing;joint10;deadline" TIMEOUT 10)
add_executable(m1_deadline_tests m1_deadline_tests.c ../tools/m1_homing/m1_deadline.c)
target_include_directories(m1_deadline_tests PRIVATE hal_stubs/m1_deadline ../tools/m1_homing)
add_test(NAME m1_deadline_tests COMMAND m1_deadline_tests)
set_tests_properties(m1_deadline_tests PROPERTIES LABELS "stage3;homing;deadline" TIMEOUT 10)
add_executable(m1_homing_logic_tests m1_homing_logic_tests.c ../tools/m1_homing/m1_logic.c)
add_executable(m1_motor10_logic_tests m1_homing_logic_tests.c ../tools/m1_homing/m1_logic.c)
target_compile_definitions(m1_motor10_logic_tests PRIVATE M1_JOG_MOTOR_TEN_DEG=1)
target_include_directories(m1_motor10_logic_tests PRIVATE ../tools/m1_homing)
add_test(NAME m1_motor10_logic_tests COMMAND m1_motor10_logic_tests)
set_tests_properties(m1_motor10_logic_tests PROPERTIES LABELS "stage3;homing;motor10" TIMEOUT 10)
add_executable(m1_homing_rx_tests m1_homing_rx_tests.c ../tools/m1_homing/m1_rx.c)
target_include_directories(m1_homing_rx_tests PRIVATE ../tools/m1_homing)
add_test(NAME m1_homing_rx_tests COMMAND m1_homing_rx_tests)
set_tests_properties(m1_homing_rx_tests PROPERTIES LABELS "stage3;homing;serial" TIMEOUT 10)
target_include_directories(m1_homing_logic_tests PRIVATE ../tools/m1_homing)
add_test(NAME m1_homing_logic_tests COMMAND m1_homing_logic_tests)
set_tests_properties(m1_homing_logic_tests PROPERTIES LABELS "stage3;homing;single_axis" TIMEOUT 10)
add_executable(tmc_bench_tests tmc_bench_tests.c ../tools/tmc_bench/bench_logic.c)
add_test(NAME tmc_bench_tests COMMAND tmc_bench_tests)
set_tests_properties(tmc_bench_tests PROPERTIES LABELS "stage7;tmc;bench" TIMEOUT 10)
add_executable(platform_time_tests platform_time_tests.c ../RobotArm/Platform/platform_time.c
    ../RobotArm/Service/safety/limit_irq_guard.c
    ../RobotArm/Service/homing/svc_homing.c)
target_include_directories(platform_time_tests PRIVATE hal_stubs/time ../RobotArm/Platform
    ../RobotArm/Service/include ../RobotArm/Device/include ../RobotArm/Algorithm/include/common)
add_test(NAME platform_time_tests COMMAND platform_time_tests)
set_tests_properties(platform_time_tests PROPERTIES LABELS "stage0;time" TIMEOUT 10)
add_library(fake_robot STATIC support/fake_robot.c
    ../RobotArm/Algorithm/common/robot_home_pose.c)
target_include_directories(fake_robot PUBLIC support
    ../RobotArm/Device/include ../RobotArm/Platform
    ../RobotArm/Algorithm/include ../RobotArm/Algorithm/include/common)

add_executable(motion_engine_tests motion_engine_tests.c
    ../RobotArm/Service/motion/internal/ctrl_motion_engine.c)
target_include_directories(motion_engine_tests PRIVATE
    ../RobotArm/Service/include ../RobotArm/Service/motion/internal)
target_link_libraries(motion_engine_tests PRIVATE fake_robot m)
add_test(NAME motion_engine_tests COMMAND motion_engine_tests)
set_tests_properties(motion_engine_tests PROPERTIES LABELS "stage2;motion" TIMEOUT 10)
add_executable(uart_line_reader_tests uart_line_reader_tests.c
    ../RobotArm/BSP/src/uart_line_reader.c)
target_include_directories(uart_line_reader_tests PRIVATE ../RobotArm/BSP/include)
add_test(NAME uart_line_reader_tests COMMAND uart_line_reader_tests)
set_tests_properties(uart_line_reader_tests PROPERTIES LABELS "stage1;serial" TIMEOUT 10)
add_executable(bsp_uart_tests bsp_uart_tests.c
    ../RobotArm/BSP/src/bsp_uart1.c ../RobotArm/BSP/src/uart_line_reader.c)
target_include_directories(bsp_uart_tests PRIVATE hal_stubs ../RobotArm/BSP/include)
add_test(NAME bsp_uart_tests COMMAND bsp_uart_tests)
foreach(scenario IN ITEMS deferred_wrap discard_wrap completed_wrap deferred_overwrite)
    add_test(NAME bsp_uart_${scenario} COMMAND bsp_uart_tests ${scenario})
    set_tests_properties(bsp_uart_${scenario} PROPERTIES LABELS "stage1;serial;dma_coherence" TIMEOUT 10)
endforeach()
set_tests_properties(bsp_uart_tests PROPERTIES LABELS "stage1;serial" TIMEOUT 10)

add_executable(homing_service_tests homing_service_tests.c
    ../RobotArm/Service/safety/limit_irq_guard.c
    ../RobotArm/Service/homing/svc_homing.c)
target_include_directories(homing_service_tests PRIVATE ../RobotArm/Service/include)
target_link_libraries(homing_service_tests PRIVATE fake_robot m)
add_test(NAME homing_service_tests COMMAND homing_service_tests)
set_tests_properties(homing_service_tests PROPERTIES LABELS "stage3;homing" TIMEOUT 10)
add_executable(homing_direction_override_tests homing_service_tests.c
    ../RobotArm/Service/homing/svc_homing.c ../RobotArm/Service/safety/limit_irq_guard.c)
target_include_directories(homing_direction_override_tests PRIVATE ../RobotArm/Service/include)
target_compile_definitions(homing_direction_override_tests PRIVATE
    HOMING_SEEK_M1_DIR_HIGH=1 HOMING_SEEK_M2_DIR_HIGH=1 HOMING_SEEK_M3_DIR_HIGH=0)
target_link_libraries(homing_direction_override_tests PRIVATE fake_robot m)
add_test(NAME homing_direction_override_tests COMMAND homing_direction_override_tests)
set_tests_properties(homing_direction_override_tests PROPERTIES LABELS "stage3;homing" TIMEOUT 10)

# 真实三层代码一直运行到HAL写引脚，验证DIR电平不被CW兼容语义反转。
foreach(mode IN ITEMS default override)
    add_executable(homing_gpio_${mode}_tests homing_gpio_contract_tests.c
        ../RobotArm/Service/homing/svc_homing.c ../RobotArm/Device/src/dev_joint.c
        ../RobotArm/BSP/driver/src/drv_stepper.c ../RobotArm/Algorithm/common/robot_home_pose.c)
    target_include_directories(homing_gpio_${mode}_tests PRIVATE hal_stubs/direction
        ../RobotArm/Service/include ../RobotArm/Device/include ../RobotArm/BSP/include
        ../RobotArm/BSP/driver/include ../RobotArm/Algorithm/include/common ../RobotArm/Platform)
    target_link_libraries(homing_gpio_${mode}_tests PRIVATE m)
    if(mode STREQUAL "override")
        target_compile_definitions(homing_gpio_${mode}_tests PRIVATE TEST_OVERRIDE_PHYSICAL_DIR=1
            HOMING_SEEK_M1_DIR_HIGH=1 HOMING_SEEK_M2_DIR_HIGH=1 HOMING_SEEK_M3_DIR_HIGH=0)
    endif()
    add_test(NAME homing_gpio_${mode}_tests COMMAND homing_gpio_${mode}_tests)
    set_tests_properties(homing_gpio_${mode}_tests PROPERTIES LABELS "stage3;homing;gpio" TIMEOUT 10)
endforeach()

# 高层全部使用生产源码；替身仅位于明确的 Platform/Device/AppHardware 端口。
add_library(host_core STATIC
    ../RobotArm/Service/safety/limit_irq_guard.c
    ../RobotArm/Service/diagnostics/limit_observation_service.c
    ../RobotArm/Service/waveform/waveform_service.c
    ../RobotArm/App/app_waveform.c
    ../RobotArm/Algorithm/common/robot_math.c
    ../RobotArm/Algorithm/kinematics/kinematics.c
    ../RobotArm/Service/command/command_service.c
    ../RobotArm/Service/planning/planning_service.c
    ../RobotArm/Service/planning/internal/ctrl_planner.c
    ../RobotArm/Service/motion/motion_service.c
    ../RobotArm/Service/motion/internal/ctrl_motion_engine.c
    ../RobotArm/Service/motion/internal/ctrl_closed_loop.c
    ../RobotArm/Service/state/state_service.c
    ../RobotArm/Service/safety/safety_service.c
    ../RobotArm/Service/homing/svc_homing.c
    ../RobotArm/Service/gripper/svc_gripper.c
    ../RobotArm/App/app_gcode_parser.c
    ../RobotArm/App/app_gcode_exec.c
    ../RobotArm/App/app_serial_protocol.c
    ../RobotArm/App/app_diagnostics.c
    ../RobotArm/App/app_calibration.c
    ../RobotArm/App/app_teleop.c
    ../RobotArm/Platform/compat/os/os_adapter.c)
target_include_directories(host_core PUBLIC
    ../RobotArm/App/include ../RobotArm/App/adapters
    ../RobotArm/Service/include ../RobotArm/Service/motion/internal
    ../RobotArm/Service/planning/internal ../RobotArm/Platform/compat)
target_link_libraries(host_core PUBLIC fake_robot m)
# 离线预演仅存在于主机工程：设备/时间由测试端口替换，绝不加入ARM固件。
add_executable(robotarm_plan_check ../tools/host/offline_plan_check.c support/fake_hardware.c)
target_link_libraries(robotarm_plan_check PRIVATE host_core)
add_executable(startup_trial_app_tests startup_trial_app_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c ../RobotArm/Service/homing/svc_homing.c)
target_compile_definitions(startup_trial_app_tests PRIVATE ROBOTARM_STARTUP_TRIAL_MODE=1
    HOMING_STEP_DELAY_US=20000 HOMING_MAX_STEPS=200)
target_link_libraries(startup_trial_app_tests PRIVATE host_core)
foreach(scenario IN ITEMS success noseek stuck fault)
    add_test(NAME startup_trial_app_${scenario} COMMAND startup_trial_app_tests ${scenario})
    set_tests_properties(startup_trial_app_${scenario} PROPERTIES LABELS "stage5;startup;bounded" TIMEOUT 10)
endforeach()

add_executable(closed_loop_feedback_timing_tests closed_loop_feedback_timing_tests.c)
target_link_libraries(closed_loop_feedback_timing_tests PRIVATE host_core)
target_link_options(closed_loop_feedback_timing_tests PRIVATE -Wl,--wrap=Dev_Joint_ReadMotorAngle)
add_test(NAME closed_loop_feedback_timing_tests COMMAND closed_loop_feedback_timing_tests)
set_tests_properties(closed_loop_feedback_timing_tests PROPERTIES LABELS "stage4;feedback" TIMEOUT 10)

add_executable(state_feedback_tests state_feedback_tests.c)
target_link_libraries(state_feedback_tests PRIVATE host_core)
foreach(scenario IN ITEMS axes invalid wrap slow)
    add_test(NAME state_feedback_${scenario}_tests COMMAND state_feedback_tests ${scenario})
    set_tests_properties(state_feedback_${scenario}_tests PROPERTIES LABELS "stage4;feedback" TIMEOUT 10)
endforeach()
add_executable(feedback_protocol_tests feedback_protocol_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c)
target_compile_definitions(feedback_protocol_tests PRIVATE ROBOTARM_DIAGNOSTIC_MODE=0 ROBOTARM_WAVEFORM_MODE=0)
target_link_libraries(feedback_protocol_tests PRIVATE host_core)
foreach(mode RANGE 0 6)
    add_test(NAME feedback_protocol_${mode}_tests COMMAND feedback_protocol_tests ${mode})
    set_tests_properties(feedback_protocol_${mode}_tests PROPERTIES LABELS "stage5;feedback;protocol" TIMEOUT 10)
endforeach()

add_executable(gripper_protocol_tests gripper_protocol_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c)
target_compile_definitions(gripper_protocol_tests PRIVATE ROBOTARM_DIAGNOSTIC_MODE=0 ROBOTARM_WAVEFORM_MODE=0)
target_link_libraries(gripper_protocol_tests PRIVATE host_core)
foreach(mode RANGE 0 6)
    add_test(NAME gripper_protocol_${mode} COMMAND gripper_protocol_tests ${mode})
    set_tests_properties(gripper_protocol_${mode} PROPERTIES LABELS "stage5;gripper;protocol" TIMEOUT 10)
endforeach()

foreach(name IN ITEMS firmware_integration_tests diagnostic_runtime_tests waveform_runtime_tests)
    add_executable(${name} ${name}.c support/fake_hardware.c ../RobotArm/App/app_runtime.c)
    target_link_libraries(${name} PRIVATE host_core)
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES LABELS "stage5;integration" TIMEOUT 15)
endforeach()
target_compile_definitions(firmware_integration_tests PRIVATE ROBOTARM_DIAGNOSTIC_MODE=0)
add_executable(homing_logging_tests homing_logging_tests.c support/fake_hardware.c ../RobotArm/App/app_runtime.c)
target_compile_definitions(homing_logging_tests PRIVATE ROBOTARM_DIAGNOSTIC_MODE=0)
target_link_libraries(homing_logging_tests PRIVATE host_core)
target_link_options(homing_logging_tests PRIVATE -Wl,--wrap=printf -Wl,--wrap=puts)
add_test(NAME homing_logging_tests COMMAND homing_logging_tests)
set_tests_properties(homing_logging_tests PROPERTIES LABELS "stage5;homing" TIMEOUT 15)

add_executable(startup_readiness_tests startup_readiness_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c)
target_compile_definitions(startup_readiness_tests PRIVATE ROBOTARM_DIAGNOSTIC_MODE=0)
target_link_libraries(startup_readiness_tests PRIVATE host_core)
add_test(NAME startup_readiness_tests COMMAND startup_readiness_tests)
set_tests_properties(startup_readiness_tests PROPERTIES LABELS "stage5;startup" TIMEOUT 15)
foreach(scenario IN ITEMS gripper_hold_homing gripper_failure gripper_recovery timeout stop recovery stale rollover deadline nonfinite)
    add_test(NAME startup_${scenario}_tests COMMAND startup_readiness_tests ${scenario})
    set_tests_properties(startup_${scenario}_tests PROPERTIES LABELS "stage5;startup" TIMEOUT 15)
endforeach()
target_compile_definitions(diagnostic_runtime_tests PRIVATE ROBOTARM_DIAGNOSTIC_MODE=1)

# 单独编译最小诊断入口，其他协议/服务仍链接相同生产源码。
add_executable(diagnostic_minimal_runtime_tests diagnostic_minimal_runtime_tests.c
    support/fake_hardware.c ../RobotArm/App/app_runtime.c ../RobotArm/App/app_diagnostics.c
    ../RobotArm/App/adapters/app_isr_adapter.c)
target_include_directories(diagnostic_minimal_runtime_tests PRIVATE hal_stubs/isr)
target_compile_definitions(diagnostic_minimal_runtime_tests PRIVATE
    ROBOTARM_DIAGNOSTIC_MODE=1 ROBOTARM_DIAGNOSTIC_ENCODERS=0)
target_link_libraries(diagnostic_minimal_runtime_tests PRIVATE host_core)
add_test(NAME diagnostic_minimal_runtime_tests COMMAND diagnostic_minimal_runtime_tests)
set_tests_properties(diagnostic_minimal_runtime_tests PROPERTIES LABELS "stage5;diagnostics" TIMEOUT 15)
add_executable(limit_observation_tests limit_observation_tests.c
    ../RobotArm/Service/diagnostics/limit_observation_service.c)
target_include_directories(limit_observation_tests PRIVATE ../RobotArm/Service/include)
target_link_libraries(limit_observation_tests PRIVATE fake_robot)
add_test(NAME limit_observation_tests COMMAND limit_observation_tests)
set_tests_properties(limit_observation_tests PROPERTIES LABELS "stage5;diagnostics" TIMEOUT 10)
target_compile_definitions(waveform_runtime_tests PRIVATE ROBOTARM_DIAGNOSTIC_MODE=0 ROBOTARM_WAVEFORM_MODE=1)
set_tests_properties(waveform_runtime_tests PROPERTIES LABELS "stage6;waveform" TIMEOUT 15)
add_executable(waveform_service_tests waveform_service_tests.c
    ../RobotArm/Service/safety/limit_irq_guard.c
    ../RobotArm/Service/waveform/waveform_service.c)
target_include_directories(waveform_service_tests PRIVATE ../RobotArm/Service/include)
target_link_libraries(waveform_service_tests PRIVATE fake_robot)
add_test(NAME waveform_service_tests COMMAND waveform_service_tests)
set_tests_properties(waveform_service_tests PROPERTIES LABELS "stage6;waveform" TIMEOUT 15)
add_executable(hardware_adapter_tests hardware_adapter_tests.c
    ../RobotArm/App/adapters/app_hardware_adapter.c)
target_include_directories(hardware_adapter_tests PRIVATE ../RobotArm/BSP/driver/include
    ../RobotArm/App/adapters ../RobotArm/Device/include ../RobotArm/BSP/include
    ../RobotArm/Algorithm/include/common ../RobotArm/Service/include ../RobotArm/Platform)
add_test(NAME hardware_adapter_tests COMMAND hardware_adapter_tests)
set_tests_properties(hardware_adapter_tests PROPERTIES LABELS "stage5;integration" TIMEOUT 10)
foreach(name IN ITEMS command_service_tests closed_loop_tests state_and_teleop_tests)
    add_executable(${name} ${name}.c support/fake_hardware.c)
    target_link_libraries(${name} PRIVATE host_core)
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES LABELS "stage4;service" TIMEOUT 15)
endforeach()

add_executable(robotarm_logic_tests
    logic_tests.c
    ../RobotArm/App/app_gcode_parser.c
    ../RobotArm/Algorithm/kinematics/kinematics.c
    ../RobotArm/Algorithm/common/robot_math.c
)

target_include_directories(robotarm_logic_tests PRIVATE
    ../RobotArm/App/include
    ../RobotArm/Service/include
    ../RobotArm/Algorithm/include
    ../RobotArm/Algorithm/include/common
)

if(MINGW OR UNIX)
    target_link_libraries(robotarm_logic_tests PRIVATE m)
endif()

add_test(NAME robotarm_logic_tests COMMAND robotarm_logic_tests)

add_executable(planning_service_tests
    planning_service_tests.c
    ../RobotArm/Service/planning/planning_service.c
    ../RobotArm/Service/planning/internal/ctrl_planner.c
    ../RobotArm/Algorithm/kinematics/kinematics.c
    ../RobotArm/Algorithm/common/robot_math.c
)

target_include_directories(planning_service_tests PRIVATE
    ../RobotArm/Platform
    ../RobotArm/Service/include
    ../RobotArm/Service/planning/internal
    ../RobotArm/Algorithm/include
    ../RobotArm/Algorithm/include/common
)

if(MINGW OR UNIX)
    target_link_libraries(planning_service_tests PRIVATE m)
endif()

add_test(NAME planning_service_tests COMMAND planning_service_tests)

add_executable(safety_service_tests
    safety_service_tests.c
    ../RobotArm/Service/safety/safety_service.c
)

target_include_directories(safety_service_tests PRIVATE
    ../RobotArm/Service/include
    ../RobotArm/Platform
)

add_test(NAME safety_service_tests COMMAND safety_service_tests)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(REAL_HAL_EXTI_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/hal_exti_handler_generated.c")
add_custom_command(OUTPUT "${REAL_HAL_EXTI_SOURCE}"
    COMMAND ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/support/extract_hal_exti.py
        ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h
        "${REAL_HAL_EXTI_SOURCE}"
    DEPENDS support/extract_hal_exti.py
        ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
        ../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h
    VERBATIM)
add_executable(real_hal_limit_irq_tests real_hal_limit_irq_tests.c "${REAL_HAL_EXTI_SOURCE}"
    ../RobotArm/BSP/src/bsp_limit_irq.c ../RobotArm/Service/safety/limit_irq_guard.c)
target_include_directories(real_hal_limit_irq_tests PRIVATE hal_stubs/limit_irq
    ../RobotArm/BSP/include ../RobotArm/Service/include ../RobotArm/Platform)
add_test(NAME real_hal_limit_irq_tests COMMAND real_hal_limit_irq_tests)
set_tests_properties(real_hal_limit_irq_tests PROPERTIES LABELS "stage5;irq_guard;HAL;integration" TIMEOUT 10)
add_test(NAME sorting_preview_tests COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/sorting_preview_tests.py $<TARGET_FILE:robotarm_plan_check>)
set_tests_properties(sorting_preview_tests PROPERTIES LABELS "stage5;python;planning;integration" TIMEOUT 30)
foreach(name IN ITEMS task_preflight_tests irq_evidence_parser_tests gripper_status_tests host_client_tests architecture_checker_tests jog_trace_report_tests axis_home_default_guard_tests acceptance_material_tests)
    add_test(NAME ${name} COMMAND ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/${name}.py)
    set_tests_properties(${name} PROPERTIES LABELS "stage5;python" TIMEOUT 20)
endforeach()
add_test(NAME irq_axis_parser_tests COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/irq_axis_parser_tests.py $<TARGET_FILE:irq_axis_evidence_tests>)
set_tests_properties(irq_axis_parser_tests PROPERTIES LABELS "stage5;irq_guard;evidence;python" TIMEOUT 15)
add_test(NAME architecture_dependencies COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/check_architecture_dependencies.py)
set_tests_properties(architecture_dependencies PROPERTIES LABELS "stage0;architecture" TIMEOUT 20)
set_tests_properties(robotarm_logic_tests PROPERTIES LABELS "stage0;algorithm" TIMEOUT 10)
set_tests_properties(planning_service_tests safety_service_tests PROPERTIES LABELS "stage4;service" TIMEOUT 10)

# 每个阶段提供自己的构建依赖和测试入口，课堂可单独运行该阶段。
foreach(stage RANGE 0 7)
    add_custom_target(check_stage${stage}
        COMMAND ${CMAKE_CTEST_COMMAND} --test-dir ${CMAKE_BINARY_DIR}
            --output-on-failure -L "^stage${stage}$"
        USES_TERMINAL VERBATIM)
endforeach()

add_executable(tmc2209_driver_tests tmc2209_driver_tests.c
    ../RobotArm/BSP/driver/src/drv_tmc2209.c)
target_include_directories(tmc2209_driver_tests PRIVATE hal_stubs/tmc ../RobotArm/BSP/driver/include)
add_test(NAME tmc2209_driver_tests COMMAND tmc2209_driver_tests)
set_tests_properties(tmc2209_driver_tests PROPERTIES LABELS "stage7;driver" TIMEOUT 10)
add_executable(tmc2209_bus_model_tests tmc2209_bus_model_tests.c
    ../RobotArm/BSP/driver/src/drv_tmc2209.c)
target_include_directories(tmc2209_bus_model_tests PRIVATE hal_stubs/tmc ../RobotArm/BSP/driver/include)
add_test(NAME tmc2209_bus_model_tests COMMAND tmc2209_bus_model_tests)
set_tests_properties(tmc2209_bus_model_tests PROPERTIES LABELS "stage7;driver" TIMEOUT 10)
add_dependencies(check_stage7 tmc2209_bus_model_tests)
foreach(scenario IN ITEMS nodes spread_high wrap preserve dropped tx_failure crc echo readback spread_changed missing arguments null_report)
    add_test(NAME tmc_quiet_${scenario} COMMAND tmc2209_bus_model_tests ${scenario})
    set_tests_properties(tmc_quiet_${scenario} PROPERTIES LABELS "stage7;driver;quiet" TIMEOUT 10)
endforeach()
add_executable(startup_driver_tests startup_driver_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c)
target_compile_definitions(startup_driver_tests PRIVATE ROBOTARM_DIAGNOSTIC_MODE=0 ROBOTARM_WAVEFORM_MODE=0)
target_link_libraries(startup_driver_tests PRIVATE host_core)
add_test(NAME startup_driver_tests COMMAND startup_driver_tests)
set_tests_properties(startup_driver_tests PROPERTIES LABELS "stage5;stage7;startup" TIMEOUT 15)
add_dependencies(check_stage5 startup_driver_tests)
add_dependencies(check_stage7 startup_driver_tests)
add_executable(startup_irq_boundary_tests startup_irq_boundary_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c)
target_link_libraries(startup_irq_boundary_tests PRIVATE host_core)
target_compile_definitions(startup_irq_boundary_tests PRIVATE ROBOTARM_DIAGNOSTIC_MODE=0 ROBOTARM_WAVEFORM_MODE=0)
target_link_options(startup_irq_boundary_tests PRIVATE
    -Wl,--wrap=AppHardware_TryInitGripper -Wl,--wrap=AppHardware_EnableJoints
    -Wl,--wrap=Svc_Homing_Start -Wl,--wrap=Svc_Homing_Step)
foreach(point IN ITEMS gripper_return enable_return home_start home_step)
    foreach(axis RANGE 0 2)
        add_test(NAME startup_irq_boundary_${point}_${axis} COMMAND startup_irq_boundary_tests ${point} ${axis})
        set_tests_properties(startup_irq_boundary_${point}_${axis} PROPERTIES LABELS "stage5;startup;irq_guard;boundary" TIMEOUT 10)
    endforeach()
endforeach()
add_dependencies(check_stage5 startup_irq_boundary_tests)
add_executable(startup_feedback_irq_tests startup_feedback_irq_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c)
target_link_libraries(startup_feedback_irq_tests PRIVATE host_core)
target_compile_definitions(startup_feedback_irq_tests PRIVATE ROBOTARM_DIAGNOSTIC_MODE=0 ROBOTARM_WAVEFORM_MODE=0)
target_link_options(startup_feedback_irq_tests PRIVATE -Wl,--wrap=AppHardware_ReadMotorAngle)
foreach(scenario IN ITEMS valid invalid deadline recovery)
    foreach(axis RANGE 0 2)
        add_test(NAME startup_feedback_irq_${scenario}_${axis} COMMAND startup_feedback_irq_tests ${scenario} ${axis})
        set_tests_properties(startup_feedback_irq_${scenario}_${axis} PROPERTIES LABELS "stage5;startup;irq_guard;boundary" TIMEOUT 10)
    endforeach()
endforeach()
add_dependencies(check_stage5 startup_feedback_irq_tests)
add_executable(startup_finalize_irq_tests startup_finalize_irq_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c)
target_link_libraries(startup_finalize_irq_tests PRIVATE host_core)
target_compile_definitions(startup_finalize_irq_tests PRIVATE ROBOTARM_DIAGNOSTIC_MODE=0 ROBOTARM_WAVEFORM_MODE=0)
target_link_options(startup_finalize_irq_tests PRIVATE
    -Wl,--wrap=PlatformCritical_Exit -Wl,--wrap=App_Calibration_Execute
    -Wl,--wrap=SafetyService_MarkHomed -Wl,--wrap=SafetyService_ClearAfterSuccessfulHoming
    -Wl,--wrap=MotionService_ClearFault -Wl,--wrap=MotionService_SetLimitMonitoring
    -Wl,--wrap=AppHardware_StartStepTimer -Wl,--wrap=AppHardware_DiscardRx -Wl,--wrap=AppSerial_SetMode)
foreach(point IN ITEMS calibration_ok calibration_fail reference_mark reference_clear motion_clear monitor timer discard)
    foreach(axis RANGE 0 2)
        add_test(NAME startup_finalize_irq_${point}_${axis} COMMAND startup_finalize_irq_tests ${point} ${axis})
        set_tests_properties(startup_finalize_irq_${point}_${axis} PROPERTIES LABELS "stage5;startup;irq_guard;boundary" TIMEOUT 10)
    endforeach()
endforeach()
foreach(control IN ITEMS success plain_failure)
    add_test(NAME startup_finalize_irq_${control} COMMAND startup_finalize_irq_tests ${control} 0)
    set_tests_properties(startup_finalize_irq_${control} PROPERTIES LABELS "stage5;startup;irq_guard;boundary" TIMEOUT 10)
endforeach()
foreach(point IN ITEMS calibration_ok calibration_fail)
    foreach(axis RANGE 0 2)
        add_test(NAME startup_finalize_irq_recovery_${point}_${axis} COMMAND startup_finalize_irq_tests ${point} ${axis} recovery)
        set_tests_properties(startup_finalize_irq_recovery_${point}_${axis} PROPERTIES LABELS "stage5;startup;irq_guard;boundary" TIMEOUT 10)
    endforeach()
endforeach()
add_dependencies(check_stage5 startup_finalize_irq_tests)
add_executable(startup_audit_tests startup_audit_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c ../RobotArm/App/app_serial_protocol.c)
target_compile_definitions(startup_audit_tests PRIVATE ROBOTARM_STARTUP_AUDIT_MODE=1
    ROBOTARM_DIAGNOSTIC_MODE=0 ROBOTARM_WAVEFORM_MODE=0)
target_link_libraries(startup_audit_tests PRIVATE host_core)
target_link_options(startup_audit_tests PRIVATE -Wl,--wrap=App_GCodeExec_Service)
foreach(scenario IN ITEMS normal driver timeout stop)
    add_test(NAME startup_audit_${scenario}_tests COMMAND startup_audit_tests ${scenario})
    set_tests_properties(startup_audit_${scenario}_tests PROPERTIES LABELS "stage5;startup;audit" TIMEOUT 15)
endforeach()
add_dependencies(check_stage5 startup_audit_tests)
add_executable(irq_evidence_audit_tests startup_audit_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c ../RobotArm/App/app_serial_protocol.c)
target_compile_definitions(irq_evidence_audit_tests PRIVATE ROBOTARM_STARTUP_AUDIT_MODE=1
    ROBOTARM_IRQ_EVIDENCE_MODE=1 ROBOTARM_DIAGNOSTIC_MODE=0 ROBOTARM_WAVEFORM_MODE=0)
target_link_libraries(irq_evidence_audit_tests PRIVATE host_core)
target_link_options(irq_evidence_audit_tests PRIVATE -Wl,--wrap=App_GCodeExec_Service)
add_test(NAME irq_evidence_audit_no_motion COMMAND irq_evidence_audit_tests normal)
set_tests_properties(irq_evidence_audit_no_motion PROPERTIES LABELS "stage5;startup;audit" TIMEOUT 15)
add_dependencies(check_stage5 irq_evidence_audit_tests)
set_property(TEST hardware_adapter_tests waveform_runtime_tests APPEND PROPERTY LABELS stage7)
add_dependencies(check_stage7 tmc2209_driver_tests hardware_adapter_tests waveform_runtime_tests)
add_dependencies(check_stage0 robotarm_logic_tests platform_time_tests)
add_dependencies(check_stage1 uart_line_reader_tests bsp_uart_tests)
add_dependencies(check_stage2 motion_engine_tests)
add_dependencies(check_stage3 homing_service_tests)
add_dependencies(check_stage3 homing_direction_override_tests)
add_dependencies(check_stage3 homing_gpio_default_tests homing_gpio_override_tests)
add_dependencies(check_stage3 m1_homing_logic_tests)
add_dependencies(check_stage3 m1_motor10_logic_tests)
add_dependencies(check_stage3 m1_deadline_tests)
add_dependencies(check_stage3 m1_limits_tests)
add_dependencies(check_stage3 m1_reference_tests)
add_dependencies(check_stage3 axis_binding_tests)
add_dependencies(check_stage3 axis_limit_home_pipeline_tests)
add_dependencies(check_stage3 servo_driver_tests)
add_dependencies(check_stage3 gripper_logic_tests)
add_executable(gripper_result_chain_tests gripper_result_chain_tests.c
    ../RobotArm/Service/gripper/svc_gripper.c ../RobotArm/Device/src/dev_gripper.c
    ../RobotArm/BSP/driver/src/drv_servo.c)
target_include_directories(gripper_result_chain_tests PRIVATE hal_stubs/servo
    ../RobotArm/Service/include ../RobotArm/Device/include ../RobotArm/BSP/include
    ../RobotArm/BSP/driver/include ../RobotArm/Algorithm/include
    ../RobotArm/Algorithm/include/common ../RobotArm/Platform)
target_link_libraries(gripper_result_chain_tests PRIVATE m)
foreach(scenario IN ITEMS status init_failure init_success unbound permission start_failure stop_failure wrap success)
    add_test(NAME gripper_result_${scenario} COMMAND gripper_result_chain_tests ${scenario})
    set_tests_properties(gripper_result_${scenario} PROPERTIES LABELS "stage3;gripper;integration" TIMEOUT 10)
endforeach()
add_dependencies(check_stage3 gripper_result_chain_tests)
add_dependencies(check_stage3 m1_joint10_logic_tests m1_joint10_deadline_tests)
add_dependencies(check_stage3 m1_homing_rx_tests)
add_dependencies(check_stage4 command_service_tests closed_loop_tests state_and_teleop_tests
    planning_service_tests safety_service_tests)
add_dependencies(check_stage4 state_feedback_tests)
add_dependencies(check_stage4 closed_loop_feedback_timing_tests)
add_dependencies(check_stage5 feedback_protocol_tests)

# 断能/故障后参考撤销，协议不能继续把旧关节坐标声明为有效。
add_executable(reference_protocol_tests reference_protocol_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c)
target_link_libraries(reference_protocol_tests PRIVATE host_core)
foreach(scenario IN ITEMS unhomed valid revoked fault restored raw not_ready)
    add_test(NAME reference_protocol_${scenario} COMMAND reference_protocol_tests ${scenario})
    set_tests_properties(reference_protocol_${scenario} PROPERTIES LABELS "stage5;reference;protocol" TIMEOUT 10)
endforeach()
add_dependencies(check_stage5 reference_protocol_tests)
add_dependencies(check_stage5 real_hal_limit_irq_tests)
add_dependencies(check_stage5 robotarm_plan_check)
add_dependencies(check_stage5 firmware_integration_tests diagnostic_runtime_tests hardware_adapter_tests startup_readiness_tests homing_logging_tests)
add_dependencies(check_stage5 diagnostic_minimal_runtime_tests limit_observation_tests)
add_dependencies(check_stage6 waveform_service_tests waveform_runtime_tests)
add_executable(limit_policy_comparison_tests limit_policy_comparison_tests.c
    ../RobotArm/Service/safety/limit_irq_guard.c
    ../RobotArm/Service/motion/internal/ctrl_motion_engine.c
    ../RobotArm/Service/waveform/waveform_service.c)
target_include_directories(limit_policy_comparison_tests PRIVATE
    ../RobotArm/Service/include ../RobotArm/Service/motion/internal)
target_link_libraries(limit_policy_comparison_tests PRIVATE fake_robot m)
add_test(NAME limit_policy_comparison_tests COMMAND limit_policy_comparison_tests)
set_tests_properties(limit_policy_comparison_tests PROPERTIES LABELS "stage6;diagnostics" TIMEOUT 10)
add_dependencies(check_stage6 limit_policy_comparison_tests)
foreach(mode RANGE 0 1)
    set(name isr_routing_${mode}_tests)
    add_executable(${name} isr_routing_tests.c ../RobotArm/App/adapters/app_isr_adapter.c)
    target_include_directories(${name} PRIVATE hal_stubs/isr
        ../RobotArm/BSP/include
        ../RobotArm/App/adapters ../RobotArm/App/include ../RobotArm/Service/include
        ../RobotArm/Algorithm/include ../RobotArm/Algorithm/include/common)
    target_compile_definitions(${name} PRIVATE ROBOTARM_WAVEFORM_MODE=${mode})
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES LABELS "stage6;waveform" TIMEOUT 10)
    add_dependencies(check_stage6 ${name})
endforeach()

add_executable(isr_routing_diagnostic_tests isr_routing_tests.c ../RobotArm/App/adapters/app_isr_adapter.c)
target_include_directories(isr_routing_diagnostic_tests PRIVATE hal_stubs/isr
    ../RobotArm/App/adapters ../RobotArm/App/include ../RobotArm/Service/include
    ../RobotArm/Algorithm/include ../RobotArm/Algorithm/include/common)
target_compile_definitions(isr_routing_diagnostic_tests PRIVATE ROBOTARM_WAVEFORM_MODE=0 ROBOTARM_DIAGNOSTIC_MODE=1)
add_test(NAME isr_routing_diagnostic_tests COMMAND isr_routing_diagnostic_tests)
set_tests_properties(isr_routing_diagnostic_tests PROPERTIES LABELS "stage5;diagnostics" TIMEOUT 10)
add_dependencies(check_stage5 isr_routing_diagnostic_tests)

add_executable(limit_irq_port_tests limit_irq_port_tests.c
    ../RobotArm/BSP/src/bsp_limit_irq.c ../RobotArm/Device/src/dev_limit_switch.c)
target_include_directories(limit_irq_port_tests PRIVATE hal_stubs/limit_irq
    ../RobotArm/BSP/include ../RobotArm/Device/include ../RobotArm/Algorithm/include/common)
add_test(NAME limit_irq_port_tests COMMAND limit_irq_port_tests)
set_tests_properties(limit_irq_port_tests PROPERTIES LABELS "stage5;diagnostics" TIMEOUT 10)
add_dependencies(check_stage5 limit_irq_port_tests)

add_executable(i2c_recovery_tests i2c_recovery_tests.c ../RobotArm/BSP/src/bsp_i2c_recovery.c)
target_include_directories(i2c_recovery_tests PRIVATE hal_stubs/i2c_recovery
    ../RobotArm/BSP/include ../RobotArm/Platform)
add_test(NAME i2c_recovery_tests COMMAND i2c_recovery_tests)
set_tests_properties(i2c_recovery_tests PROPERTIES LABELS "stage5;driver" TIMEOUT 10)
add_executable(as5600_driver_tests as5600_driver_tests.c
    ../RobotArm/BSP/driver/src/drv_as5600.c ../RobotArm/Algorithm/common/robot_math.c)
target_include_directories(as5600_driver_tests PRIVATE hal_stubs/i2c_recovery
    ../RobotArm/BSP/driver/include ../RobotArm/BSP/include ../RobotArm/Platform
    ../RobotArm/Algorithm/include ../RobotArm/Algorithm/include/common)
target_link_libraries(as5600_driver_tests PRIVATE m)
add_test(NAME as5600_driver_tests COMMAND as5600_driver_tests)
set_tests_properties(as5600_driver_tests PROPERTIES LABELS "stage5;driver" TIMEOUT 10)
add_dependencies(check_stage5 i2c_recovery_tests as5600_driver_tests)

add_executable(limit_irq_guard_tests limit_irq_guard_tests.c ../RobotArm/Service/safety/limit_irq_guard.c)
target_include_directories(limit_irq_guard_tests PRIVATE ../RobotArm/Service/include)
target_link_libraries(limit_irq_guard_tests PRIVATE fake_robot)
add_test(NAME limit_irq_guard_tests COMMAND limit_irq_guard_tests)
set_tests_properties(limit_irq_guard_tests PROPERTIES LABELS "stage5;irq_guard" TIMEOUT 10)
add_dependencies(check_stage5 limit_irq_guard_tests)
add_executable(stop_actuators_tests stop_actuators_tests.c ../RobotArm/BSP/src/bsp_robot_board.c)
target_include_directories(stop_actuators_tests PRIVATE hal_stubs/stop ../RobotArm/BSP/include
    ../RobotArm/BSP/driver/include ../RobotArm/Algorithm/include/common)
add_test(NAME stop_actuators_tests COMMAND stop_actuators_tests)
set_tests_properties(stop_actuators_tests PROPERTIES LABELS "stage5;irq_guard" TIMEOUT 10)
add_dependencies(check_stage5 stop_actuators_tests)
foreach(mode RANGE 0 1)
    set(name irq_guard_runtime_${mode}_tests)
    add_executable(${name} irq_guard_integration_tests.c support/fake_hardware.c
        ../RobotArm/App/app_runtime.c ../RobotArm/App/adapters/app_isr_adapter.c)
    target_compile_definitions(${name} PRIVATE ROBOTARM_DIAGNOSTIC_MODE=0 ROBOTARM_WAVEFORM_MODE=${mode})
    target_include_directories(${name} PRIVATE hal_stubs/isr)
    target_include_directories(${name} PRIVATE ../RobotArm/BSP/include)
    target_link_libraries(${name} PRIVATE host_core)
    if(mode EQUAL 0)
        foreach(scenario IN ITEMS early checking homing ready)
            add_test(NAME irq_guard_${scenario}_tests COMMAND ${name} ${scenario})
            set_tests_properties(irq_guard_${scenario}_tests PROPERTIES LABELS "stage5;irq_guard" TIMEOUT 15)
        endforeach()
    else()
        add_test(NAME irq_guard_waveform_tests COMMAND ${name} waveform)
        set_tests_properties(irq_guard_waveform_tests PROPERTIES LABELS "stage5;irq_guard" TIMEOUT 15)
    endif()
    add_dependencies(check_stage5 ${name})
endforeach()

add_executable(limit_vector_trace_tests limit_vector_trace_tests.c ../RobotArm/BSP/src/bsp_limit_trace.c)
target_include_directories(limit_vector_trace_tests PRIVATE hal_stubs/limit_trace ../RobotArm/BSP/include)
add_test(NAME limit_vector_trace_tests COMMAND limit_vector_trace_tests)
set_tests_properties(limit_vector_trace_tests PROPERTIES LABELS "stage5;diagnostics" TIMEOUT 10)
add_dependencies(check_stage5 limit_vector_trace_tests)

add_executable(action_permission_tests action_permission_tests.c support/fake_hardware.c ../RobotArm/App/app_runtime.c)
target_compile_definitions(action_permission_tests PRIVATE ROBOTARM_DIAGNOSTIC_MODE=0 ROBOTARM_WAVEFORM_MODE=0)
target_link_libraries(action_permission_tests PRIVATE host_core)
foreach(scenario IN ITEMS gripper_start_failure unhomed command_fault gripper_fault serial_fault qualified_limit direct_limit direct_gripper_limit motion_fault ps2_fault ps2_feedback ps2_switch recovery_success wait_reply correction_range correction_timeout correction_stalled)
    add_test(NAME action_${scenario}_tests COMMAND action_permission_tests ${scenario})
    set_tests_properties(action_${scenario}_tests PROPERTIES LABELS "stage5;permission" TIMEOUT 15)
endforeach()
add_dependencies(check_stage5 action_permission_tests)
add_executable(teleop_lifecycle_tests teleop_lifecycle_tests.c support/fake_hardware.c ../RobotArm/App/app_runtime.c)
target_compile_definitions(teleop_lifecycle_tests PRIVATE ROBOTARM_DIAGNOSTIC_MODE=0 ROBOTARM_WAVEFORM_MODE=0)
target_link_libraries(teleop_lifecycle_tests PRIVATE host_core)
foreach(scenario IN ITEMS target single neutral loss loss_recovery loss_recovery_held mode_switch_recovery)
    add_test(NAME teleop_${scenario}_tests COMMAND teleop_lifecycle_tests ${scenario})
    set_tests_properties(teleop_${scenario}_tests PROPERTIES LABELS "stage5;teleop" TIMEOUT 15)
endforeach()
add_dependencies(check_stage5 teleop_lifecycle_tests)

add_executable(teleop_coordinate_handoff_tests teleop_coordinate_handoff_tests.c)
target_link_libraries(teleop_coordinate_handoff_tests PRIVATE host_core)
foreach(scenario IN ITEMS x y z feed_only queued rejected tiny)
    add_test(NAME teleop_coordinate_handoff_${scenario} COMMAND teleop_coordinate_handoff_tests ${scenario})
    set_tests_properties(teleop_coordinate_handoff_${scenario} PROPERTIES LABELS "stage5;teleop;protocol" TIMEOUT 15)
endforeach()
add_dependencies(check_stage5 teleop_coordinate_handoff_tests)

add_executable(closed_loop_teleop_completion_tests closed_loop_teleop_completion_tests.c support/fake_hardware.c)
target_link_libraries(closed_loop_teleop_completion_tests PRIVATE host_core)
target_link_options(closed_loop_teleop_completion_tests PRIVATE -Wl,--wrap=AppHardware_ReadTeleopInput -Wl,--wrap=Dev_Joint_Step)
foreach(scenario IN ITEMS entry_stick entry_button reentry reinitialize gcode_no_poll conflicting_buttons
                          deadband direction_positive direction_negative bounded no_progress divergence small_correction stop_recovery)
    add_test(NAME completion_${scenario} COMMAND closed_loop_teleop_completion_tests ${scenario})
    set_tests_properties(completion_${scenario} PROPERTIES LABELS "stage5;teleop;closed_loop" TIMEOUT 15)
endforeach()
add_dependencies(check_stage5 closed_loop_teleop_completion_tests)

add_executable(closed_loop_window_tests closed_loop_window_tests.c)
target_link_libraries(closed_loop_window_tests PRIVATE host_core)
target_link_options(closed_loop_window_tests PRIVATE -Wl,--wrap=Dev_Joint_Step)
foreach(scenario IN ITEMS edge_0_pos edge_0_neg edge_1_pos edge_1_neg edge_2_pos edge_2_neg
    outside_0_pos outside_0_neg outside_1_pos outside_1_neg outside_2_pos outside_2_neg
    converge partial wrap_converge deadband stalled small_stalled divergence arrival_before_stall
    timeout_busy timeout_settle wrap_timeout raw_window busy_window settle_window
    sync_resets disable_resets encoder soft_limit limit normal_busy)
    add_test(NAME recovery_window_${scenario} COMMAND closed_loop_window_tests ${scenario})
    set_tests_properties(recovery_window_${scenario} PROPERTIES LABELS "stage5;closed_loop;recovery_window" TIMEOUT 15)
endforeach()
add_dependencies(check_stage5 closed_loop_window_tests)

add_executable(limit_probe_logic_tests limit_probe_logic_tests.c ../tools/limit_probe/probe_logic.c)
target_include_directories(limit_probe_logic_tests PRIVATE ../tools/limit_probe)
add_test(NAME limit_probe_logic_tests COMMAND limit_probe_logic_tests)
set_tests_properties(limit_probe_logic_tests PROPERTIES LABELS "stage5;diagnostics" TIMEOUT 10)
add_dependencies(check_stage5 limit_probe_logic_tests)

add_executable(waveform_encoder_mask_tests waveform_encoder_mask_tests.c support/fake_hardware.c ../RobotArm/App/app_runtime.c)
target_compile_definitions(waveform_encoder_mask_tests PRIVATE ROBOTARM_DIAGNOSTIC_MODE=0 ROBOTARM_WAVEFORM_MODE=1)
target_link_libraries(waveform_encoder_mask_tests PRIVATE host_core)
add_test(NAME waveform_encoder_mask_tests COMMAND waveform_encoder_mask_tests)
set_tests_properties(waveform_encoder_mask_tests PROPERTIES LABELS "stage6;diagnostics" TIMEOUT 10)
add_dependencies(check_stage6 waveform_encoder_mask_tests)

add_executable(limit_probe_console_tests limit_probe_console_tests.c ../tools/limit_probe/probe_console.c)
target_include_directories(limit_probe_console_tests PRIVATE ../tools/limit_probe)
add_test(NAME limit_probe_console_tests COMMAND limit_probe_console_tests)
set_tests_properties(limit_probe_console_tests PROPERTIES LABELS "stage5;diagnostics" TIMEOUT 10)
add_dependencies(check_stage5 limit_probe_console_tests)
add_dependencies(check_stage3 startup_trial_logic_tests)
add_dependencies(check_stage5 startup_trial_app_tests)
add_dependencies(check_stage5 irq_evidence_tests)
add_dependencies(check_stage5 irq_axis_evidence_tests)

add_executable(seek_contact_irq_tests seek_contact_irq_tests.c support/fake_hardware.c
    ../RobotArm/App/adapters/app_isr_adapter.c
    ../tools/limit_seek_trial/seek_logic.c ../tools/limit_seek_trial/contact_gate.c
    ../tools/limit_seek_trial/boot_contact.c)
target_include_directories(seek_contact_irq_tests PRIVATE hal_stubs/isr ../tools/limit_seek_trial)
target_compile_definitions(seek_contact_irq_tests PRIVATE ROBOTARM_LIMIT_SEEK_TRIAL=1
    ROBOTARM_DIAGNOSTIC_MODE=0 ROBOTARM_WAVEFORM_MODE=0 SEEK_MAX_STEPS=1500U
    SEEK_AXIS_TIMEOUT_MS=30000U SEEK_ALLOW_PRESSED_START=1)
target_link_libraries(seek_contact_irq_tests PRIVATE host_core)
foreach(scenario IN ITEMS old contact bounce unstable poll high other inactive threshold latched_other sequence wrap repeat_deadline boot_high boot_threshold boot_pressed boot_bounce boot_timeout boot_all)
    add_test(NAME seek_contact_${scenario} COMMAND seek_contact_irq_tests ${scenario})
    set_tests_properties(seek_contact_${scenario} PROPERTIES LABELS "stage5;irq_guard;contact" TIMEOUT 10)
endforeach()
add_executable(seek_result_tests seek_result_tests.c ../tools/limit_seek_trial/seek_result.c)
target_include_directories(seek_result_tests PRIVATE ../tools/limit_seek_trial ../tools/startup_trial)
target_link_libraries(seek_result_tests PRIVATE host_core)
add_test(NAME seek_result_tests COMMAND seek_result_tests)
add_dependencies(check_stage5 seek_contact_irq_tests seek_result_tests)
add_executable(seek_input_hold_tests seek_input_hold_tests.c
    ../tools/limit_seek_trial/input_hold.c ../tools/limit_seek_trial/seek_logic.c)
target_include_directories(seek_input_hold_tests PRIVATE ../tools/limit_seek_trial)
target_compile_definitions(seek_input_hold_tests PRIVATE SEEK_ALLOW_PRESSED_START=1
    SEEK_MAX_STEPS=1500U SEEK_AXIS_TIMEOUT_MS=30000U)
foreach(scenario IN ITEMS glitch release unstable deadline abort wrap)
    add_test(NAME seek_hold_${scenario} COMMAND seek_input_hold_tests ${scenario})
    set_tests_properties(seek_hold_${scenario} PROPERTIES LABELS "stage5;contact" TIMEOUT 10)
endforeach()
add_dependencies(check_stage5 seek_input_hold_tests)

add_executable(home_sequence_tests home_sequence_tests.c ../RobotArm/Algorithm/common/home_sequence.c)
target_include_directories(home_sequence_tests PRIVATE ../RobotArm/Algorithm/include)
foreach(scenario IN ITEMS sequence pressed full_scale contact_bounce unstable seek_timeout total_timeout stuck no_ack rearm_denied unexpected hold hold_release retrigger boot_glitch wrap first_cause recovery_release config)
    add_test(NAME home_sequence_${scenario} COMMAND home_sequence_tests ${scenario})
    set_tests_properties(home_sequence_${scenario} PROPERTIES LABELS "stage3;homing;sequence" TIMEOUT 10)
endforeach()
add_dependencies(check_stage3 home_sequence_tests)

# 克隆同一份生产上层源码，只切换回零候选宏；244项原有测试仍测试原入口。
get_target_property(seq_core_sources host_core SOURCES)
add_library(host_core_sequence STATIC ${seq_core_sources}
    ../RobotArm/Algorithm/common/home_sequence.c ../RobotArm/Service/homing/svc_homing_sequence.c)
get_target_property(seq_core_includes host_core INCLUDE_DIRECTORIES)
target_include_directories(host_core_sequence PUBLIC ${seq_core_includes})
target_compile_definitions(host_core_sequence PUBLIC ROBOTARM_SEQUENTIAL_HOMING=1)
target_link_libraries(host_core_sequence PUBLIC fake_robot m)
add_executable(sequence_runtime_tests sequence_runtime_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c ../RobotArm/App/adapters/app_isr_adapter.c)
target_include_directories(sequence_runtime_tests PRIVATE hal_stubs/isr)
target_link_libraries(sequence_runtime_tests PRIVATE host_core_sequence)
target_link_options(sequence_runtime_tests PRIVATE -Wl,--wrap=AppHardware_EnableJoints -Wl,--wrap=Dev_Joint_SetFeedbackZero)
foreach(scenario IN ITEMS normal boot_pressed recovery gripper_fail calibration_fail stuck rearm_fail rearm_recovery retrigger irq early_irq stall contact_flood calibration_irq handoff_irq handoff_limit)
    add_test(NAME sequence_runtime_${scenario} COMMAND sequence_runtime_tests ${scenario})
    set_tests_properties(sequence_runtime_${scenario} PROPERTIES LABELS "stage5;homing;integration" TIMEOUT 15)
endforeach()
add_dependencies(check_stage5 sequence_runtime_tests)

# 171生产并行候选：真实App/Service/算法，显式替换GPIO、驱动和时间。
get_target_property(parallel_sources host_core_sequence SOURCES)
add_library(host_core_parallel STATIC ${parallel_sources} ../RobotArm/Algorithm/common/limit_state_filter.c)
target_include_directories(host_core_parallel PUBLIC ${seq_core_includes})
target_compile_definitions(host_core_parallel PUBLIC ROBOTARM_SEQUENTIAL_HOMING=1 ROBOTARM_LIMIT_STATE_MODE=1 ROBOTARM_PARALLEL_HOMING=1)
target_link_libraries(host_core_parallel PUBLIC fake_robot m)

add_executable(limit_indicator_tick_tests limit_indicator_tick_tests.c support/fake_hardware.c
    ../RobotArm/App/adapters/app_isr_adapter.c)
target_include_directories(limit_indicator_tick_tests PRIVATE hal_stubs/isr)
target_link_libraries(limit_indicator_tick_tests PRIVATE host_core_parallel)
add_test(NAME limit_indicator_tick_tests COMMAND limit_indicator_tick_tests)
set_tests_properties(limit_indicator_tick_tests PROPERTIES LABELS "stage5;gpio;indicator" TIMEOUT 10)
add_dependencies(check_stage5 limit_indicator_tick_tests)

add_executable(mode_switch_visibility_tests mode_switch_visibility_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c ../RobotArm/App/adapters/app_isr_adapter.c)
target_include_directories(mode_switch_visibility_tests PRIVATE hal_stubs/isr)
target_link_libraries(mode_switch_visibility_tests PRIVATE host_core_parallel)
foreach(scenario IN ITEMS switch_pair fault_mode recovery_indicator moving_refused)
    add_test(NAME mode_switch_visibility_${scenario} COMMAND mode_switch_visibility_tests ${scenario})
    set_tests_properties(mode_switch_visibility_${scenario} PROPERTIES LABELS "stage5;mode;indicator" TIMEOUT 15)
endforeach()
add_dependencies(check_stage5 mode_switch_visibility_tests)
add_executable(parallel_runtime_tests parallel_runtime_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c ../RobotArm/App/adapters/app_isr_adapter.c)
target_include_directories(parallel_runtime_tests PRIVATE hal_stubs/isr)
target_link_libraries(parallel_runtime_tests PRIVATE host_core_parallel)
target_link_options(parallel_runtime_tests PRIVATE -Wl,--wrap=Dev_Joint_WriteStepLevel)
foreach(scenario IN ITEMS normal pressed port stuck recovery rate1000)
    add_test(NAME parallel_runtime_${scenario} COMMAND parallel_runtime_tests ${scenario})
    set_tests_properties(parallel_runtime_${scenario} PROPERTIES LABELS "stage5;homing;parallel" TIMEOUT 15)
endforeach()
add_executable(parallel_homing_tests parallel_homing_tests.c ../RobotArm/Algorithm/common/home_sequence.c)
target_include_directories(parallel_homing_tests PRIVATE ../RobotArm/Algorithm/include)
foreach(scenario IN ITEMS initial0 initial1 initial2 initial3 initial4 initial5 initial6 initial7
    stuck deny scale wrap barrier glitch gap unstable confirmed_release deadline cancel retrigger_glitch retrigger_held back_gap)
    add_test(NAME parallel_homing_${scenario} COMMAND parallel_homing_tests ${scenario})
    set_tests_properties(parallel_homing_${scenario} PROPERTIES LABELS "stage3;homing;parallel" TIMEOUT 10)
endforeach()
add_executable(ps2_diagnostic_query_tests ps2_diagnostic_query_tests.c support/fake_hardware.c)
target_link_libraries(ps2_diagnostic_query_tests PRIVATE host_core)
add_test(NAME ps2_diagnostic_query COMMAND ps2_diagnostic_query_tests)
set_tests_properties(ps2_diagnostic_query PROPERTIES LABELS "stage5;ps2;protocol;readonly" TIMEOUT 10)
add_dependencies(check_stage3 parallel_homing_tests)
add_dependencies(check_stage5 parallel_runtime_tests ps2_diagnostic_query_tests)

# GPIO候选复用相同生产App/Service，保留原279项；只替换设备/时钟端口。
get_target_property(state_core_sources host_core_sequence SOURCES)
add_library(host_core_state STATIC ${state_core_sources} ../RobotArm/Algorithm/common/limit_state_filter.c)
target_include_directories(host_core_state PUBLIC ${seq_core_includes})
target_compile_definitions(host_core_state PUBLIC ROBOTARM_SEQUENTIAL_HOMING=1 ROBOTARM_LIMIT_STATE_MODE=1)
target_link_libraries(host_core_state PUBLIC fake_robot m)
add_executable(state_runtime_tests sequence_runtime_tests.c support/fake_hardware.c
    ../RobotArm/App/app_runtime.c ../RobotArm/App/adapters/app_isr_adapter.c)
target_include_directories(state_runtime_tests PRIVATE hal_stubs/isr)
target_link_libraries(state_runtime_tests PRIVATE host_core_state)
target_link_options(state_runtime_tests PRIVATE -Wl,--wrap=AppHardware_EnableJoints -Wl,--wrap=Dev_Joint_SetFeedbackZero)
foreach(scenario IN ITEMS normal boot_pressed recovery gripper_fail calibration_fail stuck rearm_fail rearm_recovery retrigger stall handoff_limit state_noise state_latched state_stop state_bounce state_unstable runtime_press runtime_glitch runtime_unstable runtime_recovery runtime_stop_fail other_glitch other_held other_unstable other_backoff contact_glitch contact_repeat seek_no30s seek_total_deadline ready_hold release_reference lifecycle_move lifecycle_gripper lifecycle_stop lifecycle_recovery lifecycle_driver_retry)
    add_test(NAME gpio_runtime_${scenario} COMMAND state_runtime_tests ${scenario})
    set_tests_properties(gpio_runtime_${scenario} PROPERTIES LABELS "stage5;homing;gpio_state" TIMEOUT 15)
endforeach()
add_executable(limit_state_filter_tests limit_state_filter_tests.c ../RobotArm/Algorithm/common/limit_state_filter.c)
target_include_directories(limit_state_filter_tests PRIVATE ../RobotArm/Algorithm/include)
foreach(scenario IN ITEMS held glitch bounce unstable gap repeat wrap reset)
    add_test(NAME gpio_filter_${scenario} COMMAND limit_state_filter_tests ${scenario})
    set_tests_properties(gpio_filter_${scenario} PROPERTIES LABELS "stage3;gpio_state" TIMEOUT 10)
endforeach()
add_executable(limit_state_port_tests limit_state_port_tests.c
    ../RobotArm/BSP/src/bsp_limit_irq.c ../RobotArm/Device/src/dev_limit_switch.c)
target_include_directories(limit_state_port_tests PRIVATE hal_stubs/limit_irq
    ../RobotArm/BSP/include ../RobotArm/Device/include ../RobotArm/Algorithm/include/common)
target_compile_definitions(limit_state_port_tests PRIVATE ROBOTARM_LIMIT_STATE_MODE=1)
add_test(NAME gpio_state_port COMMAND limit_state_port_tests)
add_dependencies(check_stage3 limit_state_filter_tests)
add_dependencies(check_stage5 state_runtime_tests limit_state_port_tests)
add_executable(home_sequence_pause_tests home_sequence_pause_tests.c ../RobotArm/Algorithm/common/home_sequence.c)
target_include_directories(home_sequence_pause_tests PRIVATE ../RobotArm/Algorithm/include)
foreach(scenario IN ITEMS recorded38 many_pauses pause_cap total_deadline seek_deadline wrap)
    add_test(NAME home_pause_${scenario} COMMAND home_sequence_pause_tests ${scenario})
    set_tests_properties(home_pause_${scenario} PROPERTIES LABELS "stage3;homing;pause_budget" TIMEOUT 10)
endforeach()
add_dependencies(check_stage3 home_sequence_pause_tests)
add_executable(home_sequence_speed_tests home_sequence_speed_tests.c ../RobotArm/Algorithm/common/home_sequence.c)
target_include_directories(home_sequence_speed_tests PRIVATE ../RobotArm/Algorithm/include)
foreach(scenario IN ITEMS normal pressed wrap pause timeout one_ms bad1001 fallback)
    add_test(NAME home_speed_${scenario} COMMAND home_sequence_speed_tests ${scenario})
    set_tests_properties(home_speed_${scenario} PROPERTIES LABELS "stage3;homing;speed" TIMEOUT 10)
endforeach()
add_dependencies(check_stage3 home_sequence_speed_tests)
add_executable(home_other_input_tests home_other_input_tests.c ../RobotArm/Algorithm/common/home_sequence.c)
target_include_directories(home_other_input_tests PRIVATE ../RobotArm/Algorithm/include)
foreach(scenario IN ITEMS glitch2 glitch3 pulse9 held unstable gap repeat wrap backoff rearmed completed deadline)
    add_test(NAME home_other_${scenario} COMMAND home_other_input_tests ${scenario})
    set_tests_properties(home_other_${scenario} PROPERTIES LABELS "stage3;homing;gpio_state" TIMEOUT 10)
endforeach()
add_dependencies(check_stage3 home_other_input_tests)
add_executable(home_contact_cancel_tests home_contact_cancel_tests.c ../RobotArm/Algorithm/common/home_sequence.c)
target_include_directories(home_contact_cancel_tests PRIVATE ../RobotArm/Algorithm/include)
foreach(scenario IN ITEMS glitch low9 held bounce deadline total gap wrap)
    add_test(NAME home_contact_${scenario} COMMAND home_contact_cancel_tests ${scenario})
    set_tests_properties(home_contact_${scenario} PROPERTIES LABELS "stage3;homing;gpio_state" TIMEOUT 10)
endforeach()
add_dependencies(check_stage3 home_contact_cancel_tests)
add_executable(home_seek500_tests home_seek500_tests.c ../RobotArm/Algorithm/common/home_sequence.c)
target_include_directories(home_seek500_tests PRIVATE ../RobotArm/Algorithm/include)
foreach(scenario IN ITEMS normal no30s nonzero total invalid wrap)
    add_test(NAME home_seek500_${scenario} COMMAND home_seek500_tests ${scenario})
    set_tests_properties(home_seek500_${scenario} PROPERTIES LABELS "stage3;homing;speed" TIMEOUT 10)
endforeach()
add_dependencies(check_stage3 home_seek500_tests)
add_executable(home1000_tests home1000_tests.c ../RobotArm/Algorithm/common/home_sequence.c)
target_include_directories(home1000_tests PRIVATE ../RobotArm/Algorithm/include)
foreach(scenario IN ITEMS single parallel wrap same_ms gap first_low barrier deadline cancel bad_zero bad_slow fallback)
    add_test(NAME home1000_${scenario} COMMAND home1000_tests ${scenario})
    set_tests_properties(home1000_${scenario} PROPERTIES LABELS "stage3;homing;speed" TIMEOUT 10)
endforeach()
add_dependencies(check_stage3 home1000_tests)
add_executable(serial_reply_flow_tests serial_reply_flow_tests.c support/fake_hardware.c)
target_link_libraries(serial_reply_flow_tests PRIVATE host_core)
foreach(scenario IN ITEMS queries burst ordered settled)
    add_test(NAME serial_reply_flow_${scenario} COMMAND serial_reply_flow_tests ${scenario})
    set_tests_properties(serial_reply_flow_${scenario} PROPERTIES LABELS "stage5;serial;protocol" TIMEOUT 10)
endforeach()
add_dependencies(check_stage5 serial_reply_flow_tests)

add_executable(ready_hold_tests ready_hold_tests.c ../tools/startup_trial/ready_hold.c)
target_include_directories(ready_hold_tests PRIVATE ../tools/startup_trial)
foreach(scenario IN ITEMS config normal wrap unhomed revoked fault same_tick_fault)
    add_test(NAME ready_hold_${scenario} COMMAND ready_hold_tests ${scenario})
    set_tests_properties(ready_hold_${scenario} PROPERTIES LABELS "stage3;startup;holding" TIMEOUT 10)
endforeach()
add_dependencies(check_stage3 ready_hold_tests)

add_executable(ps2_driver_tests ps2_driver_tests.c ../RobotArm/BSP/driver/src/drv_ps2.c)
target_include_directories(ps2_driver_tests PRIVATE ../RobotArm/BSP/driver/include ../RobotArm/Platform)
foreach(scenario IN ITEMS timing null mapping header pressure digital cooldown wrap diag_valid diag_header diag_unknown diag_latch diag_reset)
    add_test(NAME ps2_driver_${scenario} COMMAND ps2_driver_tests ${scenario})
    set_tests_properties(ps2_driver_${scenario} PROPERTIES LABELS "stage3;ps2;driver" TIMEOUT 10)
endforeach()
foreach(scenario IN ITEMS acquire_success acquire_timeout acquire_then_loss held_then_loss conflict_then_loss acquire_queued)
    add_test(NAME completion_${scenario} COMMAND closed_loop_teleop_completion_tests ${scenario})
    set_tests_properties(completion_${scenario} PROPERTIES LABELS "stage5;ps2;acquisition" TIMEOUT 10)
endforeach()
add_test(NAME completion_alternating_buttons COMMAND closed_loop_teleop_completion_tests alternating_buttons)
set_tests_properties(completion_alternating_buttons PROPERTIES LABELS "stage5;ps2;gripper;rapid" TIMEOUT 10)
add_dependencies(check_stage3 ps2_driver_tests)

# 闭环判定只读快照：真实控制/STEP/串口生产代码，设备与时间端口用替身。
add_executable(closed_loop_status_tests closed_loop_status_tests.c support/fake_hardware.c)
target_link_libraries(closed_loop_status_tests PRIVATE host_core)
target_link_options(closed_loop_status_tests PRIVATE -Wl,--wrap=Dev_Joint_ReadMotorAngle -Wl,--wrap=Dev_Joint_Step -Wl,--wrap=MotionService_GetClosedLoopStatus)
foreach(scenario IN ITEMS init deadband small recovery stop sync failed_read age_wrap ps2 disabled syntax bounds mode0 mode1 mode2 mode3 mode4 mode5 mode6)
    add_test(NAME closed_loop_status_${scenario} COMMAND closed_loop_status_tests ${scenario})
    set_tests_properties(closed_loop_status_${scenario} PROPERTIES LABELS "stage5;closed_loop;protocol" TIMEOUT 10)
endforeach()
add_dependencies(check_stage5 closed_loop_status_tests)
add_executable(teleop_continuity_tests teleop_continuity_tests.c support/fake_hardware.c)
target_link_libraries(teleop_continuity_tests PRIVATE host_core)
target_link_options(teleop_continuity_tests PRIVATE -Wl,--wrap=MotionService_SubmitFrame)
foreach(scenario IN ITEMS continuity bound target neutral loss stop gcode invalid)
    add_test(NAME teleop_continuity_${scenario} COMMAND teleop_continuity_tests ${scenario})
    set_tests_properties(teleop_continuity_${scenario} PROPERTIES LABELS "stage5;ps2;continuity" TIMEOUT 10)
endforeach()
add_dependencies(check_stage5 teleop_continuity_tests)
foreach(scenario IN ITEMS closed_loop_boot closed_loop_recovery)
    add_test(NAME gpio_runtime_${scenario} COMMAND state_runtime_tests ${scenario})
    set_tests_properties(gpio_runtime_${scenario} PROPERTIES LABELS "stage5;closed_loop;reference" TIMEOUT 15)
endforeach()
