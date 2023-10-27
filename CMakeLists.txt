cmake_minimum_required(VERSION 3.24.0)
project(mgmecs_tests)
enable_testing(include(CTest))

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_executable(basic_ecs ${CMAKE_CURRENT_SOURCE_DIR}/tests/basic_ecs.cpp)
target_include_directories(basic_ecs PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
