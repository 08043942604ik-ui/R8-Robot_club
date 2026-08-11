#include "Lidar.h"


Lidarnode::Lidarnode() : Node("lidar") {
    RCLCPP_INFO(get_logger(), "Lidar constructor");

    const bool init_ok = lidars.initi();

    RCLCPP_INFO(
        get_logger(),
        "Lidar initialize = %s",
        init_ok ? "OK" : "FAILED");

    if (!init_ok) {
        return;
    }

    const bool start_ok = lidars.start();

    RCLCPP_INFO(
        get_logger(),
        "Lidar start = %s",
        start_ok ? "OK" : "FAILED");
}

~Lidarnode() {
lidars.End();
  
}

double Lidarnode::disGetter() {
if (laser_.doProcessSimple(raw_data_)) {
        
distanceByWalls = raw_data_.ranges;

}

double SendDBW(double k) {

return k;

} 



void timer_callback() {
    if (!laser_.isScanning()) return;

if(dud) {
lidars::initi();
lidars::start 
disGetter();

SendDBW(distanceByWalls);

    }


}
