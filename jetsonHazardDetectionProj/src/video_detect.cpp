#include <jetson-inference/detectNet.h>
#include <jetson-utils/videoSource.h>
#include <jetson-utils/videoOutput.h>
#include <jetson-utils/videoOptions.h>
#include <signal.h>
#include <stdio.h>


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
	URI uri_input = URI("v4l2:///dev/video0");
	URI uri_output = URI("display://0");
	videoOptions input_options = videoOptions();
	videoOptions output_options = videoOptions();
    //options.deviceType = deviceTypeFromStr("v4l2");
    //options.Print();
	
    videoSource* input = videoSource::Create(uri_input, input_options);
	
	videoOutput* output = videoOutput::Create(uri_output, output_options);

    detectNet* net = detectNet::Create();
    const uint32_t overlayFlags = detectNet::OverlayFlagsFromStr("box,labels,conf");
    while(!signal_recieved){
	    uchar3* image = NULL;
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

        /*if(numDetections > 0){
		    for(int n=0; n < numDetections; n++){
			
			}
		}*/

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

	SAFE_DELETE(input);
	SAFE_DELETE(output);
	SAFE_DELETE(net);
    return 0;
}

