#include <studica_drivers/servo.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <studica_drivers/dio.h>
#include <VMXPi.h>

class servom : public rclcpp::Node {
public :

servom();

~servom();

void callback();



private :



rclcpp::TimerBase::SharedPtr Stimer_;
bool cut = true;
int doe = 1;
int angle = 0;
int times = 0;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_Main;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_Main;
int count = 0;
double first_value_ = 0.0;
double second_value_ = 0.0;
double third_value_ = 0.0;

bool doit = false;
bool flag = true;
bool red1 = false;
bool red2 = false;
bool red3 = false;


};