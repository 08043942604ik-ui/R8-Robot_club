#pragma once
#include <CYdLidar.h>


class Lidarinit {
public :

Lidarinit() = default;

bool initi();
bool start();
void End();



private :
Lidar Ldr;
int baudrate = 230400;
int lidar_type = TYPE_TOF;
int sample_rate = 4;
int device_type = YDLIDAR_TYPE_SERIAL;
int abnormal_count = 4;
bool fixed_resolution = true;
float scan_frequency = 10.0;
bool auto_reconnect = true;
bool single_channel = false;
bool intensity = false;
bool reversion = false;

}
