cmake_minimum_required(VERSION 2.8)
project(hazard_detect_one)


set(RPLIDAR_SDK_PATH "./include/rplidar/")
set(SPI_SDK_PATH "./include/spi/")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall")

find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)
find_package(CUDA REQUIRED)
find_package(Threads REQUIRED)

file(GLOB RPLIDAR_SDK_SRC
    "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

file(GLOB SPI_SRC
    "${SPI_SDK_PATH}/*.cc")

include_directories(
    "${RPLIDAR_SDK_PATH}/src"
    "${RPLIDAR_SDK_PATH}/include"
    "${SPI_SDK_PATH}"
)

link_directories(/usr/lib/aarch64-linux-gnu/tegra)
include_directories(${CUDA_INCLUDE_DIRS})

add_executable(hazarddetect src/video_detect.cpp ${RPLIDAR_SDK_SRC} ${SPI_SRC})
target_link_libraries(hazarddetect PUBLIC jetson-inference jetson-utils)
target_link_libraries(hazarddetect PRIVATE Threads::Threads)
