#include "lidartest.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class Lidarnode : public rclcpp::Node {

Lidarnode();


void timer_callback();
double disGetter();
double SendDBW(double k);






private :
rclcpp::TimerBase::SharedPtr timer_;
double distanceByWalls;
Lidarinit lidars;


}
