#include "Lidar.h"


Lidarnode::Lidarnode() : Node("lidar") {




  
}















void timer_callback() {
    if (!laser_.isScanning()) return;

    // 最新のデータを受け取る（初期設定で開始角度を正面にセットしてある状態
    if (laser_.doProcessSimple(raw_data_)) {
        
        std::vector<float> front_scan = raw_data_.ranges;
