#pragma once
#include <CYdLidar.h>
#include <string>

class Lidarinit {
public :

Lidarinit() = default;

bool initi();
bool start();
void End();



private :
CYdLidar Ldr;

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
scanrangemax = 16.f; //何メートルから何メートルまでの壁を検知対象にするかという設定で
scanrangemin = 0.1f;
  


}
