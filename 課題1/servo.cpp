#include "servo.h" 
#include <chrono>

servom::servom()
    : Node("servo_node")
{
Stimer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() { callback(); });


           publisher_ = create_publisher<std_msgs::msg::Bool>("/GetOrder/Place", 10);
            publisher_Main = create_publisher<std_msgs::msg::Float64>("/Gosignal", 10);
           
           subscriber02_ = create_subscription<std_msgs::msg::Bool>(
"/set/flag", 10, 
[this](std_msgs::msg::Bool::ConstSharedPtr msg) {
flag = msg->date;
});



           subscriber_ = create_subscription<std_msgs::msg::Float64>(
    "/Take/this", 10,
    [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
     if(count == 0) {
    first_value_ = msg->data;
    RCLCPP_WARN(get_logger(), "get the first value");
    flag = false;
    if((first_value_ - 100000.0) > 0) {
     std_msgs::msg::Float64 msgred;

msgred.date = 6.7; 
publisher_Main->publish(msgred);
    }
    else if((first_value - 100000.0) < 0) {
std_msgs::msg::Float64 msggr;
msggr.date = 9.6;
publisher_Main->publish(msggr);

}




}
   

    });
}
 servom::~servom() {} 
 

 
 void servom::callback() { 

 
 

    std_msgs::msg::Bool msg;
    msg.data = flag; 
    publisher_->publish(msg);

}