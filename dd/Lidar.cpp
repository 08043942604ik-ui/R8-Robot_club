#include "Lidar.h"


Lidarnode::Lidarnode() : Node("lidar") {
lidars::initi();
lidars::start();
  
timer_ = create_wall_timer(
  std::chrono::milliseconds(20),
      [this]() { timer_callback(); });
lidars::End();
  
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
