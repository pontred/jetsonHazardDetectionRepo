cmake_minimum_required(VERSION 2.8)
project(hazard_detect)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)
find_package(CUDA REQUIRED)

link_directories(/usr/lib/aarch64-linux-gnu/tegra)
include_directories(${CUDA_INCLUDE_DIRS})

add_executable(hazarddetect src/video_detect.cpp)
target_link_libraries(hazarddetect jetson-inference jetson-utils)
