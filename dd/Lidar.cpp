#include "Lidar.h"


Lidarnode::Lidarnode() : Node("lidar") {
Lidars::initi();
Lidars::start();
  
timer_ = create_wall_timer(
  std::chrono::milliseconds(20),
      [this]() { timer_callback(); });
  
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
disGetter();

SendDBW(distanceByWalls);

    }
}
