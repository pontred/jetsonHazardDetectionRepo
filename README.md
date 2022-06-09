# jetsonHazardDetectionRepo
***

## Installation
For full Repo go [here](https://github.com/pontred/jetsonHazardDetectionRepo)

Follow the instructions here [jetson-inference](https://github.com/dusty-nv/jetson-inference/blob/master/docs/building-repo-2.md)

To enable SPI communication on the Jetson follow the guide [here](https://jetsonhacks.com/2020/05/04/spi-on-jetson-using-jetson-io/). Follow through the Jetson-IO Section and enable SPI1 (Pins 19, 21, 23, 24, 26)

## Building the Project

Requires CMake. 

To build project
    mkdir build
    cd build
    cmake ..
    make
    ./hazarddetect

You may need to run command below to enable SPI communication
    sudo modprobe spidev
