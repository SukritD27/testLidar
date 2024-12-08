#include "include/sl_lidar.h" 
#include "include/sl_lidar_driver.h"
//#include "include/rplidar.h"
#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif
#define MOTOCTL_GPIO 		18

using namespace sl;

bool ctrl_c_pressed;
void ctrlc(int) {
    ctrl_c_pressed = true;
}

int main(int argc, char** argv)
{
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Error: Failed to initialize pigpio library.\n");
        return -1;
    }
    IChannel* _channel;
    Result<IChannel*> channel = createSerialPortChannel("/dev/ttyS0", 115200);
    //Result<ISerialChannel*> channel  = createSerialPortChannel("/dev/ttyS0", 115200);
    IChannel* actual_channel = *channel;
    ILidarDriver * lidar = *createLidarDriver();
    if (!lidar) {
        fprintf(stderr, "Error: Insufficient memory, exiting.\n");
        gpioTerminate();
        return -1;
    }else {
	    printf("SUCCESS!");
    }
    auto res = lidar->connect(*channel);

    if(SL_IS_OK(res)){
        sl_lidar_response_device_info_t deviceInfo;
        res = lidar->getDeviceInfo(deviceInfo);
        if(SL_IS_OK(res)){
            printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
            deviceInfo.model,
            deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
            deviceInfo.hardware_version);
        }else{
            fprintf(stderr, "Failed to get device information from LIDAR %08x\r\n", res);
        }
    }else{
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
    }
    gpioSetMode(MOTOCTL_GPIO, PI_OUTPUT);
    gpioWrite(MOTOCTL_GPIO, PI_LOW);
    gpioWrite(MOTOCTL_GPIO, PI_HIGH);
    printf("Motor started.\n");
    
    lidar->startScan(0,1);
    signal(SIGINT, ctrlc);

    while(!ctrl_c_pressed){
	sl_lidar_response_measurement_node_hq_t nodes[8192];
    	size_t count = _countof(nodes);

    	sl_result op_result = lidar->grabScanDataHq(nodes, count);
    	if (SL_IS_OK(op_result)) {
        	lidar->ascendScanData(nodes, count);
        	for (size_t i = 0; i < count; ++i) {
        	printf("Angle: %.2f Distance: %.2f Quality: %d\n",
              		(nodes[i].angle_z_q14 * 90.0f) / 16384.0f,
              		nodes[i].dist_mm_q2 / 4.0f,
              		nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
        	}
    	}
    }

    lidar->stop();
    gpioWrite(MOTOCTL_GPIO, PI_LOW);
    printf("Motor stopped.\n");
    delete lidar;
    delete actual_channel;
    return 0;
}
