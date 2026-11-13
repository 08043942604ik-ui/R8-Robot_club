#pragma once

#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp> //realsenseのnodeと通信するため
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

class CameraControl : public rclcpp::Node
{
public:
bool onOroff = true;
    CameraControl();

void cameraOn();


    void cameraOff();

void getcolor(const sensor_msgs::msg::Image::ConstSharedPtr msginF);





void cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);


private:

    std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_; //realsenseノードと通信するため


int HMtimes = 0;
double OnceRedratio = 0.0;

rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_s;
bool runnign = false;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
bool received_flag_ = false;
int times = 0;
bool Ondone = false;
bool running = false;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};