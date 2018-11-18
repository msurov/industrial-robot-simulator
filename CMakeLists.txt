cmake_minimum_required(VERSION 3.0)

project(ursim)
enable_testing()

set(CMAKE_CXX_STANDARD 11)
find_package(Eigen3 REQUIRED Core)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(Threads REQUIRED)
find_package(jsoncpp REQUIRED)
include_directories("/usr/include/jsoncpp")
find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

add_executable(ursim
    src/formatting.h
    src/main.cpp
    src/planner.h
    src/robot_sim.h
    src/robot_sim.cpp
    src/signals.h
    src/throws.h
    src/trajectory.h
    src/trajectory.cpp
    src/joints.h
    src/math.h
    src/signals.cpp
    src/threads.h
    src/timeutils.h
    src/kinematics.h
    src/kinematics.cpp
    src/properties.h
    src/properties.cpp
)
target_link_libraries(ursim Threads::Threads jsoncpp ${roscpp_LIBRARIES})

add_executable(test_planner
    src/trajectory.h
    src/trajectory.cpp
    src/planner.h
    tests/test_planner.cpp
)
add_test(NAME test_planner COMMAND test_planner)


add_executable(test_kinematics
    src/kinematics.h
    src/kinematics.cpp
    src/properties.h
    src/properties.cpp
    tests/test_kinematics.cpp
)
target_link_libraries(test_kinematics jsoncpp)
add_test(NAME test_kinematics COMMAND test_kinematics ${PROJECT_SOURCE_DIR}/configs/ur5.cfg)

add_executable(test_transforms
    src/transforms.h
    src/properties.h
    src/properties.cpp
    src/kinematics.h
    src/kinematics.cpp
    tests/test_transforms.cpp
)
target_link_libraries(test_transforms jsoncpp)
add_test(NAME test_transforms COMMAND test_transforms)


add_executable(test_publisher
    tests/test_publisher.cpp
)
target_link_libraries(test_publisher ${roscpp_LIBRARIES})
add_test(NAME test_publisher COMMAND test_publisher)
