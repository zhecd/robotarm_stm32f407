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
