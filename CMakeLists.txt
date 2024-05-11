cmake_minimum_required(VERSION 3.27)

set( CMAKE_BUILD_TYPE Debug )

set(CMAKE_CXX_STANDARD 17)

include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(Assignment1)
add_subdirectory(Assignment2)
add_subdirectory(Assignment3)
add_subdirectory(Assignment4)
add_subdirectory(Assignment5)