/**************************************************************************************************************
 * Jetson -Inference Licensing

 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 * 
 * Please refer to include files for other licensing agreement 
 * 
 * Description:
 * This project is meant to be placed on a vehicle to help report potential hazards to 
 * other vehicles. This is done with Lidar, a camera combined with object detection, and wifi.
 *
 * This project uses lidar and a camera to determine what an object is
 * and how far away that object is. Next it will try to predict if an object is a hazard.
 * If a hazard is detected it will send hazard classifcation, object classifcation, and
 * position of the object relate to the car (currently reported left, right, front). This
 * is information is then sent out to over SPI. 
 *
 *
 * TO DO:
 * Currently the hazard detection methodology needs to be improved. It only looks at how close
 * to an object the device is and and classifcation of the device. Assuming GPS/velocity data
 * is being sent over SPI we can use this data to give a more accurate prediction of if something
 * is a hazard by prediciting time to collision given we know the devices velocity and the distance
 * from the object.
 *
 * Current SPI data is getting received, but not used. This data should be information from a seperate
 * device that is relying data from other vehicles such as hazard warnings and velocity. 
 *
 * Currently this project is not modular at all. Breaking the project down into functions will help
 * increase readability and also make modifying code less dangerous as you will be less likely to spend
 * time searching for a missing '}'.
 *
 * Currently when more than one lidar array is made the program does not load. If this problem can be
 * fixed we can create a better 2-D point map that can detect things outside of the cameras field of view.
 * This would also allow the jetson to calculate how how fast we are approaching the detected object by
 * comparing the current lidar array to the previous lidar array.
 *
 * Lasted Edited: 06/09/2022
 * Author: pontred
 * **********************************************************************************************************/
#include <jetson-inference/detectNet.h>
#include <jetson-utils/videoSource.h>
#include <jetson-utils/videoOutput.h>
#include <jetson-utils/videoOptions.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <rplidar.h>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include "spidev_lib++.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEGREE_PER_PIXEL 78/1280
#define TOLERANCE 0.5

//SPI variables
#define DUMMY_BITS 4
#define SPI_DATA_LENGTH (104 + DUMMY_BITS)
#define ASTERICK_LOCATION_TX (SPI_DATA_LENGTH - DUMMY_BITS - 3)
#define CHKSUM_MSB_LOCATION_TX (SPI_DATA_LENGTH - DUMMY_BITS - 2)
#define CHKSUM_LSB_LOCATION_TX (SPI_DATA_LENGTH - DUMMY_BITS - 1)
#define PREAMBLE_LOCATION_TX 0
#define ASTERICK_LOCATION_RX (SPI_DATA_LENGTH - DUMMY_BITS/2 - 3)
#define CHKSUM_MSB_LOCATION_RX (SPI_DATA_LENGTH - DUMMY_BITS/2 - 2)
#define CHKSUM_LSB_LOCATION_RX (SPI_DATA_LENGTH - DUMMY_BITS/2 - 1)
#define PREAMBLE_LOCATION_RX (DUMMY_BITS/2)
#define PREAMBLE 0x24
#define COMMA 0x2C
#define ASTERICK 0x2A
#define ASCII_NUMBER_MASK 0x30 
#define ASCII_LETTER_MASK 0x41

spi_config_t spi_config;

typedef enum {NO_HAZARD, CATION, STOP} HAZARD_T;
typedef enum {NO_OBJ, PERSON, ANIMAL, VEHICLE, OTHER} OBJ_T;
typedef enum {NA, LEFT, FRONT, RIGHT, BACK} OBJ_ANGLE_T;

/*****************************************************************************************
 * Data to be sent be out of the jetson to an IEC device. Each subject to be sent out will be
 * seperated by a comma. Note that the tx and rx buffer lengths need to transmit the same
 * amount of data to work properly. 
 *
 * Preamble   -  1 Byte(s) - Used to identify beginning of message
 * Hazard     -  1 Byte(s) - Indicated type of hazard. Refer to hazard enum.
 * Obj        -  1 Byte(s) - Inicates type of object. Refer to obj enum.
 * Obj_Angle  -  1 Byte(s) - Indicates where hazard is. Refer to angle enum
 * chksum     -  3 Byte(s) - '*[10's place of check sum as Char][1's place of sum as char]
 *                         - Example '*32'
 * 
 * Total ','s -  4 Byte(s)
 * Total Byte - 11 Byte(s) 
 * [PREMABLE, HAZARD, OBJ, OBJ_ANGLE, (zeros and comas until ASTERICK_LOCATION_TX), 
 *  ASTERICK_LOCATION_TX, CHKSUM_MSB_LOCATION_TX, CHKSUM_LSB_LOCATION_TX, (fill zero and commas)]
 ******************************************************************************************/
uint8_t txbuffer[SPI_DATA_LENGTH];  // Data sent to outside device


/*****************************************************************************************
 * Data to be sent to the jetson from the IEC device. Each subject recieved should be
 * seperated by a comma. Note that the tx and rx buffer lengths need to transmit the same
 * amount of data to work properly. 
 *
 * Preamble   -  1 Byte(s) - Used to identify beginning of message
 * Hazard     -  1 Byte(s) - Indicated type of hazard. Refer to hazard enum.
 * Obj        -  1 Byte(s) - Inicates type of object. Refer to obj enum.
 * Obj_Angle  -  1 Byte(s) - Indicates where hazard is. Refer to angle enum
 * Time       -  9 Byte(s) - Indicates time
 * Latitude   - 10 Byte(s) - Indicates latidude
 * Longitude  - 11 Byte(s) - Indicates longitudes
 * Speed      -  5 Byte(s) - Indicates speed in knots
 * Veh_angle  -  5 Byte(s) - Indicates relative to ##############
 * chksum     -  3 Byte(s) - '*[10's ptic determines the lifetime and visibility/accessibility of thelace of check sum as Char][1's place of sum as char
 * 
 * Total ','s -  9 Byte(s)
 * Total Byte - 56 Byte(s) 
 ******************************************************************************************/
uint8_t rxbuffer[SPI_DATA_LENGTH];

uint8_t hex_to_ascii(uint8_t chksum);
using namespace sl;

bool signal_recieved = false;
void sig_handler (int signo){
    if(signo = SIGINT){
        signal_recieved = true;
    }
}


int main(){
    if(signal(SIGINT, sig_handler) == SIG_ERR){
        printf("Signal Error\n");
    } else {
	
    }
    
// set up SPI
    spi_config.mode=0;
    spi_config.speed=500000; //1000000
    spi_config.delay=0;
    spi_config.bits_per_word=8;
    SPI* thespi =new SPI("/dev/spidev0.0", &spi_config);
// set up spi buffers with zeros   
    for(int i = 0; i < SPI_DATA_LENGTH; i++){
        txbuffer[i] = 0;
        rxbuffer[i] = 0;
    }

    uint8_t chksum = 0x00;
    uint8_t chksum_MSB = 0x00;
    uint8_t chksum_LSB = 0x00;
    HAZARD_T hazard = NO_HAZARD;
    OBJ_T obj = NO_OBJ;
    OBJ_ANGLE_T obj_angle = NA;

// lidar set up
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;
    IChannel* channel_instance;
    ILidarDriver* drv = *createLidarDriver();
    if(!drv){
        fprintf(stderr, "insufficient memeory. Exit\n");
        exit(-2);
    } else {
	
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    float runningaverage = 4000;

    channel_instance = (*createSerialPortChannel("/dev/ttyUSB0", 115200));
    if(SL_IS_OK(drv->connect(channel_instance))){
        op_result = drv->getDeviceInfo(devinfo);
        if(SL_IS_OK(op_result)){
            connectSuccess = true;
        } else {
            delete drv;
            drv = NULL;
        }
    } else {
        delete drv;
        drv = NULL;
    }


    if(connectSuccess){
        op_result = drv->getHealth(healthinfo);
        printf("SLAMTEC LIDAR HEALTH STATUS: %d\n", healthinfo.status);
        if(healthinfo.status == SL_LIDAR_STATUS_ERROR){
            fprintf(stderr, "Error, health status");
            connectSuccess = false;
            delete drv;
            drv = NULL;
        } else {
		
        }
    } else {
        connectSuccess = false;
        delete drv;
        drv = NULL;
    }

// set up camera input and out    
	URI uri_input = URI("v4l2:///dev/video0");
	URI uri_output = URI("display://0");

    videoSource* input = videoSource::Create(uri_input);
    videoOutput* output = videoOutput::Create(uri_output);

// set up detectnet instance	
    detectNet* net = detectNet::Create();
    const uint32_t overlayFlags = detectNet::OverlayFlagsFromStr("box,labels,conf");

	// set up motor
	if(connectSuccess){
        drv->setMotorSpeed();
        drv->startScan(0,1);
    } else {
    
    }
    
    while(!signal_recieved && connectSuccess){
// lidar variables
        sl_lidar_response_measurement_node_hq_t nodes [8192];
        size_t count = _countof(nodes);
// image variable
        uchar3* image = NULL;
// sensor fusion variables
        float angleleftcamera = 0;
        float anglerightcamera = 0;
        float anglewidthcamera = 0;
        float anglemidcamera = 0;
        float objectdistance = 555;
        float minimumobjectdistance = 12000;
        float lidarangle = 0;
//grab data from lidar
        op_result = drv->grabScanDataHq(nodes, count);
		if (SL_IS_OK(op_result)){
// check for close objects between -120 and +120 degrees
// take average value. current loop is above running average cation or hazard will be sent
		    drv->ascendScanData(nodes, count);
		    for(int pos = 0; pos < (int)count; ++pos){
		        lidarangle = nodes[pos].angle_z_q14*90.f/16348.f;
                if (((lidarangle < 120) && (lidarangle > 0)) | ((lidarangle > 240) && (lidarangle < 359))) {
                    runningaverage = (runningaverage + nodes[pos].dist_mm_q2)/(4.0f*2.0f);
                    objectdistance = nodes[pos].dist_mm_q2/(4.0f);
                    if ((objectdistance < minimumobjectdistance) && (objectdistance > 555)){
                        minimumobjectdistance = objectdistance;
                    } else {

                    }
                } else {

                }
            }
        } else {
    
        }
// capture image
        if(!input->Capture(&image, 1000)){
            if(!input->IsStreaming()){
                break;
            } else {

            }
            printf("Streaming Error\n");
        } else {
        
        }
// detect objects in frame
        detectNet::Detection* detections = NULL;
        const int numDetections = net->Detect(image, input->GetWidth(), input->GetWidth(), &detections, overlayFlags);
        if((numDetections > 0) && SL_IS_OK(op_result)){
            for(int n=0; n < numDetections; n++){
// convert left, right and width from pixel information to angle
// mid pixel is 0 degreees. Very left pixel is -39 deg. Very right pixel is 39 degrees. (As output on screen)
                anglerightcamera = (detections[n].Right * DEGREE_PER_PIXEL) - 39;
                angleleftcamera = (detections[n].Left * DEGREE_PER_PIXEL) - 39;
                anglewidthcamera = anglerightcamera - angleleftcamera;
// if left is negative convert to lidar angle representation (321 deg to 360 deg)
                    if (angleleftcamera < 0) {
                        angleleftcamera = 360 + angleleftcamera;
                        anglemidcamera = angleleftcamera + anglewidthcamera/2;
// the case where the left bounding box is on the left side of the image (between 321 deg and 360 deg)
// but the mid point is greater than 360 degree. midpoint angle will be between 0 deg and 39 deg.
                        if (anglemidcamera > 360) {
                            anglemidcamera = anglemidcamera - 360;
                        }
// the case where the left bound box is on the right side of the image (0 deg or above)
                    } else {
                        anglemidcamera = angleleftcamera + anglewidthcamera/2;    
                    }
// go through each angle reading from the lidar device and wait until the angle measured from the camera is 
                    for(int pos = 0; pos < (int)count; ++pos){
                        lidarangle = nodes[pos].angle_z_q14*90.f/16348.f;
                        if ((anglemidcamera > (lidarangle - TOLERANCE)) && (anglemidcamera < (lidarangle + TOLERANCE))){
                            objectdistance = nodes[pos].dist_mm_q2/4.0f;
                        } else {
                        }
                    }
// Detect type of object and hazard potential
                    if (detections[n].ClassID == PERSON){
                        txbuffer[1] = STOP;
                        txbuffer[2] = COMMA;
                        txbuffer[3] = PERSON;
                        txbuffer[4] = COMMA;
                        txbuffer[5] = FRONT;
                        txbuffer[6] = COMMA;
                    }  else if ((detections[n].ClassID >= 3) || (detections[n].ClassID <= 10)) {
                        txbuffer[1] = CATION;
                        txbuffer[2] = COMMA;
                        txbuffer[3] = VEHICLE;
                        txbuffer[4] = COMMA;
                        txbuffer[5] = FRONT;
                        txbuffer[6] = COMMA;
                    } else if ((detections[n].ClassID >= 18) || detections[n].ClassID <= 26) {
                        txbuffer[1] = CATION;
                        txbuffer[2] = COMMA;
                        txbuffer[3] = ANIMAL;
                        txbuffer[4] = COMMA;
                        txbuffer[5] = FRONT;
                        txbuffer[6] = COMMA;
                    } else {
                        txbuffer[1] = NO_HAZARD;
                        txbuffer[2] = COMMA;
                        txbuffer[3] = NO_OBJ;
                        txbuffer[4] = COMMA;
                        txbuffer[5] = FRONT;
                        txbuffer[6] = COMMA;
                    }
// Detect location of object
                    if ((anglemidcamera < 333) && (anglemidcamera > 321)) {
                        txbuffer[5] = LEFT;
                    } else if ((anglemidcamera < 39) && (anglemidcamera > 27)){
                        txbuffer[5] = RIGHT;
                    } else {
                        //front
                    }
                printf("Detection: %i, Class %u (%s), Distance: %f, Angle: %f\n", n, detections[n].ClassID, net->GetClassDesc(detections[n].ClassID), objectdistance, anglemidcamera);
                //printf("Detection: %i, Class: #%u (%s),  Right: %f, Left: %f, Width: %f, distance: %f\n", n, detections[n].ClassID, net->GetClassDesc(detections[n].ClassID), anglerightcamera, angleleftcamera, anglewidthcamera, objectdistance);
            }

// Setting up standard SPI data transfer
            txbuffer[PREAMBLE_LOCATION_TX] = PREAMBLE;
            txbuffer[ASTERICK_LOCATION_TX] = ASTERICK;
            chksum = txbuffer[1] ^ txbuffer[2] ^ txbuffer[3] ^ txbuffer[4] ^ txbuffer[5];
            txbuffer[CHKSUM_MSB_LOCATION_TX] = hex_to_ascii(((chksum >> 4) & 0x0F));
            txbuffer[CHKSUM_LSB_LOCATION_TX] = hex_to_ascii((chksum & 0x0F));
            printf("HAZARD: %X, OBJECT: %X, OBJ_ANGLE: %X", txbuffer[1], txbuffer[3], txbuffer[5]);
        } else {
        }
//render image
        if(output != NULL){
            output->Render(image, input->GetWidth(), input->GetHeight());
            char str[256];
            sprintf(str, "TensorRT %i.%i.%i | %s | Network %.0f FPS", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, precisionTypeToStr(net->GetPrecision()), net->GetNetworkFPS());
            output->SetStatus(str);
            if(!output->IsStreaming()){
                signal_recieved = true;
            }
            //net->PrintProfilerTimes();
        } else {
        }
// Read/send VIA SPI	
        if (thespi->begin()){
            thespi->xfer(txbuffer, sizeof(txbuffer)/sizeof(txbuffer[0]), rxbuffer, sizeof(rxbuffer)/sizeof(rxbuffer[0]));
        }
		//for (int i = 0; i < SPI_DATA_LENGTH; i++){
		//    printf("%c ", (char) rxbuffer[i]);
		//}
        printf("\n");
        chksum = 0;
        chksum_MSB = 0;
        chksum_LSB = 0;
// confirm PREAMBLE and *
        if ((rxbuffer[PREAMBLE_LOCATION_RX] == PREAMBLE) && (rxbuffer[ASTERICK_LOCATION_RX] == ASTERICK)) { 
// xor to create chksum
            for (int i = PREAMBLE_LOCATION_RX + 1; i < ASTERICK_LOCATION_RX; i++){
                chksum ^= rxbuffer[i];
            }
// convert check sum to ascii
            chksum_MSB = hex_to_ascii((chksum >> 4) & 0x0F);
            chksum_LSB = hex_to_ascii((chksum & 0x0f));
            //printf("msb: %c, lsb %c\n", (char) chksum_MSB, (char) chksum_LSB);
            if ((rxbuffer[CHKSUM_MSB_LOCATION_RX] == chksum_MSB) && (rxbuffer[CHKSUM_LSB_LOCATION_RX] == chksum_LSB)){
                hazard = (HAZARD_T) rxbuffer[PREAMBLE_LOCATION_RX + 1];
                obj = (OBJ_T) rxbuffer[PREAMBLE_LOCATION_RX + 3];
                obj_angle = (OBJ_ANGLE_T) rxbuffer[PREAMBLE_LOCATION_RX + 5];
                printf("From other Vehicle (Hazard: %X, Object: %X)\n", hazard, obj);
            } else {
            }
        } else {
            //printf("rx buffer error\n");
        }
        chksum = 0;
        chksum_MSB = 0;
        chksum_LSB = 0;
    }
    drv->stop();
    drv->setMotorSpeed(0);
    delete thespi;
    SAFE_DELETE(input);
    SAFE_DELETE(output);
    SAFE_DELETE(net);
    return 0;
}

/**************************************************************************************************************
 * uint8_t HexToAscii(uint8_t chksum)
 * Description: input a 4 bit hex value that 0x00 - 0x0F and get the asci 
 *representation of that value
 *
 *input: 0x00 - 0x0F
 *output: '0' through '9' and 'A' through 'F'
 *
 *Edited: pontred
 * ***********************************************************************************************************/
uint8_t hex_to_ascii (uint8_t chksum) {
    uint8_t result;
    if (chksum <= 0x9) {
        result = chksum + ASCII_NUMBER_MASK;
    } else {
        result = chksum + ASCII_LETTER_MASK - 0xa;
    }
    return result;
}
