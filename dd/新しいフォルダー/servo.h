#include <studica_drivers/servo.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class servom : public rclcpp::Node {
public :

servom();

~servom();

void callback();



private :

std::shared_ptr<studica_driver::Servo> servo_;

rclcpp::TimerBase::SharedPtr Stimer_;
bool cut = true;
int doe = 1;
int angle = 0;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};
