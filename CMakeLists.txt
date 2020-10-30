cmake_minimum_required(VERSION 3.14)
project(sx1280_driver)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_library(sx1280_driver
        sx1280.cpp
        sx1280.hpp)
