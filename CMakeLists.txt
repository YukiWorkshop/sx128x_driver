cmake_minimum_required(VERSION 3.14)
project(sx128x_driver)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_library(sx128x_driver SX128x.cpp SX128x.hpp)

add_executable(SX128x_ToA_Test ToA_Test.cpp)
target_link_libraries(SX128x_ToA_Test sx128x_driver)