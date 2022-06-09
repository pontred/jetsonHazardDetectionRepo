# jetsonHazardDetectionRepo
***
This code is intended to function on a Jetson Nano. It requires the A1M8 Lidar made by Slamtec and a camera that is v4l2 compatible. Many webcams will be compatible. Fairly simple modifications can be made to make this code compatible with other types of cameras. For more information refer to the [Jetson-inference](https://github.com/dusty-nv/jetson-inference) github. 

## Installation

This project DOES NOT have the necassary include files for the Lidar and SPI.

For the complete Repo go [here](https://github.com/pontred/jetsonHazardDetectionRepo).  

Follow the instructions here to install [jetson-inference](https://github.com/dusty-nv/jetson-inference/blob/master/docs/building-repo-2.md)

To enable SPI communication on the Jetson follow the guide [here](https://jetsonhacks.com/2020/05/04/spi-on-jetson-using-jetson-io/). Follow through to the Jetson-IO section and enable SPI1 (Pins 19, 21, 23, 24, 26).

## Building the Project

Requires CMake. 

To build project navigate to project folder in terminal and type the following commands
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./hazarddetect

You may need to run command below to enable SPI communication
- sudo modprobe spidev

