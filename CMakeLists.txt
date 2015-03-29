cmake_minimum_required(VERSION 2.6)
project(BubblesMotionPlanning)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -O -std=c++11 -Wall -Wno-unknown-pragmas")
enable_testing()
add_subdirectory(src)