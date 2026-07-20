cmake_minimum_required(VERSION 3.20)

project(robotarm_logic_tests C)

enable_testing()

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
