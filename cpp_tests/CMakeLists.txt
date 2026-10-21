cmake_minimum_required(VERSION 3.8)
project(sas_robot_driver_unitree_g1_cpp_tests)

# Tests to run on the real robot, outside ROS. Not built by colcon.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(unitree_sdk2 REQUIRED)

add_executable(g1_stand_height_test g1_stand_height_test.cpp)
target_link_libraries(g1_stand_height_test unitree_sdk2)
