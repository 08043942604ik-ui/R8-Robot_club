#pragma once
#include <CYdLidar.h>
#include <string>

class Lidarinit {
private :
CYdLidar Ldr;

bool initialized_ = false;
bool started_ = false;

std::string port = "/dev/ttyAMA0"
int baudrate = 230400; //一秒間にデータを送る速さ
std::string Ignore_array = "46, 180, -46, -180";  
int lidar_type = TYPE_TOF; //形式T-mini plusはTof
int sample_rate = 4;//kHz
int device_type = YDLIDAR_TYPE_SERIAL; //接続方式シリアルかインターネット
int abnormal_count = 4; //データが何回乱れたらエラーになるか
bool fixed_resolution = true; //レーザーの点と点の間の角度を、常に固定にするかどうかの設定
bool dyreverse = false; //データの前後を180度ひっくり返すかどうか.
float scan_frequency = 10.0; //モーターの回転スピード
bool auto_reconnect = true; //接続切れても再接続を試みるか
bool single_channel = false; //こちらから操作できるか
bool intensity = false; //レーザーの色の強さをうけとるか
float maxum = 180.0f; //何度から何度までの景色を切り取るかという設定
float minum = -180.0f;
float scanrangemax = 16.f; //何メートルから何メートルまでの壁を検知対象にするかという設定で
float scanrangemin = 0.1f;




public :

Lidarinit() = default;

bool initi() {
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


  

initialized_ = Ldr.initialize();
return initialized_;  
}

bool start() {
if (!initialized_) {
        return false;
    }

    started_ = Ldr.turnOn();
    return started_;

 

}

bool isScanning()
{
    if (!initialized_ || !started_) {
        return false;
    }

    return Ldr.isScanning();
}

bool getScan(LaserScan& scan)
{
    if (!initialized_ || !started_) {
        return false;
    }

    return Ldr.doProcessSimple(scan);
}

void stop() {
if(!isScanning) {

return;
}

Ldr.turnOff(); 
}

void End() {

    if (started_) {
        Ldr.turnOff();
        started_ = false;
    }

    if (initialized_) {
        Ldr.disconnecting();
        initialized_ = false;
    }
}





  


};
