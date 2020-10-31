cmake_minimum_required(VERSION 3.14)
project(sx128x_driver)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_library(sx128x_driver SX128x.cpp SX128x.hpp)
