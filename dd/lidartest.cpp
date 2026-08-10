#include "lidartest.h"






bool Lidarinit::initi() {
Ldr.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
Ldr.setlidaropt(LidarPropIgnoreArray, Ignore_array.c_str(), Ignore_array.size());
Ldr.setlidaropt(LidarPropBaudrate, &baudrate, sizeof(int));
Ldr.setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));
Ldr.setlidaropt(LidarPropSampleRate, &sample_rate, sizeof(int));
Ldr.setlidaropt(LidarPropDeviceType, &device_type, sizeof(int));
Ldr.setlidaropt(LidarPropAbnormalCheckCount, &abnormal_count, sizeof(int));
Ldr.setlidaropt(LidarPropFixedResolution, &fixed_resolution, sizeof(bool));
Ldr.setlidaropt(LidarPropScanFrequency, &scan_frequency, sizeof(float));
Ldr.setlidaropt(LidarPropAutoReconnect, &auto_reconnect, sizeof(bool));
Ldr.setlidaropt(LidarPropSingleChannel, &single_channel, sizeof(bool));
Ldr.setlidaropt(LidarPropIntensities, &intensity, sizeof(bool));
Ldr.setlidaropt(LidarPropReversion, &dyreverse, sizeof(bool));
Ldr.setlidaropt(LidarPropMaxAngle, &maxum, sizeof(float));
Ldr.setlidaropt(LidarPropMinAngle, &minum, sizeof(float));
Ldr.setlidaropt(LidarPropMaxRange, &scanrangemax, sizeof(float)); 
Ldr.setlidaropt(LidarPropMinRange, &scanrangemin, sizeof(float));
  
return Ldr.initialize();  
}

bool Lidarinit::start() {
req = Ldr.inititalize(); 
 
 if(req) {
Ldr.turnOn();

return true; 
 }

else {

return false;
}

}

void Lidarinit::stop() {
if(!isScanning) {

return;
}

 
}

void Lidarinit::End() {
if(!isScanning) {

return;
}

Ldr.turnOff();
Ldr.disconnecting;
}


