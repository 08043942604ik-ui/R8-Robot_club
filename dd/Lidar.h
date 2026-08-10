#include "lidartest.h"
#include <rclcpp/rclcpp.hpp>
#include <vector>

class Lidarnode : public rclcpp::Node {

Lidarnode();
~Lidarnode();

void timer_callback();
double disGetter();
double SendDBW(double k);



private :
rclcpp::TimerBase::SharedPtr timer_;
std::vector<float> distanceByWalls;
Lidarinit lidars;


}
