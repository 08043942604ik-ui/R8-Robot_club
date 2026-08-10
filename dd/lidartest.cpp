#include "lidartest.h"






bool Lidarinit::initi() {
Ldr.setlidaropt(LidarPropSerialPort, port_name.c_str(), port_name.size());
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
Ldr.setlidaropt(LidarPropReversion, &reversion, sizeof(bool));



  

  
}
