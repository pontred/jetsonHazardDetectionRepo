#include <jetson-inference/detectNet.h>
#include <jetson-utils/videoSource.h>
#include <jetson-utils/videoOutput.h>
#include <jetson-utils/videoOptions.h>
#include <signal.h>
#include <stdio.h>

#include <rplidar.h>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

static const float degree_per_pixel = 78/720;

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

	channel_instance = (*createSerialPortChannel("/dev/ttyUSB0", 115200));
	if(SL_IS_OK(drv->connect(channel_instance))){
	    op_result = drv->getDeviceInfo(devinfo);
		if(SL_IS_OK(op_result)){
		    printf("here\n");
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
    
	URI uri_input = URI("v4l2:///dev/video0");
	URI uri_output = URI("display://0");
	//videoOptions input_options = videoOptions();
	//videoOptions output_options = videoOptions();
    //input_options.Print();
	//output_options.Print();
	
    //videoSource* input = videoSource::Create(uri_input, input_options);
	videoSource* input = videoSource::Create(uri_input);
    
	//videoOutput* output = videoOutput::Create(uri_output, output_options);
    videoOutput* output = videoOutput::Create(uri_output);

	//input_options.Print();
	//output_options.Print();
	
    detectNet* net = detectNet::Create();
    const uint32_t overlayFlags = detectNet::OverlayFlagsFromStr("box,labels,conf");
	if(connectSuccess){
	    drv->setMotorSpeed();
        drv->startScan(0,1);
    } else {
	
	}
	
	while(!signal_recieved && connectSuccess){
// lidar
        sl_lidar_response_measurement_node_hq_t nodes[8192];
		size_t count = _countof(nodes);
    	op_result = drv->grabScanDataHq(nodes, count);
        
	    uchar3* image = NULL;
		float angleleftcamera = 0;
		float anglerightcamera = 0;
		float anglewidthcamera = 0;
		float objectdistance = 555;
		float lidarangle = 0;
		float tolerance = 0.5;
// capture image
		if(!input->Capture(&image, 1000)){
            if(!input->IsStreaming()){
		        break;
			} else {
			    printf("Streaming Error\n");
			}
		} else {
		
		}
// detect objects in frame
        detectNet::Detection* detections = NULL;
		const int numDetections = net->Detect(image, input->GetWidth(), input->GetWidth(), &detections, overlayFlags);

        if(numDetections > 0){
		    for(int n=0; n < numDetections; n++){
//convert left, right and width from pixel information to angle
				anglerightcamera = (detections[n].Right * 78 / 1280) - 39;
				angleleftcamera = (detections[n].Left * 78 / 1280) - 39;
				anglewidthcamera = anglerightcamera - angleleftcamera;
                if(SL_IS_OK(op_result)){
			        drv->ascendScanData(nodes,count);
		            // if left is negative convert to lidar angle representation
					if (angleleftcamera < 0) {
				        angleleftcamera = 360 + angleleftcamera;
						anglerightcamera = angleleftcamera + anglewidthcamera;
					    for(int pos = 0; pos < (int)count; ++pos){
						    lidarangle = nodes[pos].angle_z_q14*90.f/16348.f;
						    if (((angleleftcamera + anglewidthcamera/2) > (lidarangle - tolerance)) && ((angleleftcamera + anglewidthcamera/2) < (lidarangle + tolerance))){
							    objectdistance = nodes[pos].dist_mm_q2/4.0f;
							}    
					    }
					// if left is greater than 0 degrees	
				    } else {
					    for(int pos = 0; pos < (int)count; ++pos){
						    lidarangle = nodes[pos].angle_z_q14*90.f/16348.f;
						    if (((angleleftcamera + anglewidthcamera/2) > (lidarangle - tolerance)) && ((angleleftcamera + anglewidthcamera/2) < (lidarangle + tolerance))){
							    objectdistance = nodes[pos].dist_mm_q2/4.0f;
							}    
					    }
					}
				}
                printf("Detection: %d,  Right: %f, Left: %f, Width: %f, distance: %f\n", n, anglerightcamera, angleleftcamera, anglewidthcamera, objectdistance);
			}

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
			net->PrintProfilerTimes();
		}
    
	}

    drv->stop();
	drv->setMotorSpeed(0);
	
	SAFE_DELETE(input);
	SAFE_DELETE(output);
	SAFE_DELETE(net);
    return 0;
}
