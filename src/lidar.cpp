#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

int main(int argc, char* argv)
{
    ///  Create a communication channel instance
    IChannel* _channel;
    Result<ISerialChannel*> channel = createSerialPortChannel("/dev/ttyS0", 115200);
    ///  Create a LIDAR driver instance
    ILidarDriver * lidar = *createLidarDriver();
    auto res = (*lidar)->connect(*channel);
    if(SL_IS_OK(res)){
        sl_lidar_response_device_info_t deviceInfo;
        res = (*lidar)->getDeviceInfo(deviceInfo);
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
    // TODO
    lidar->startMotor();
// TODO
    for(int i = 0; i < 10; i++){
	    delay(1);
    }
    lidar->stopMotor();	
    /// Delete Lidar Driver and channel Instance
    * delete *lidar;
    * delete *channel;
}
