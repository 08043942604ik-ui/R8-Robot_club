#pragma once

#include <rclcpp/rclcpp.hpp>
#include "robotdrive.h"
#include "Encoder.h"
#include "Lidar.h"
#include "color_senser.h"

class Robotmain : public rclcpp::Node {
public:
    Robotmain()
        : Node("robot_control"),
    encoder_(std::make_shared<Encoderma>()),
      drive_(std::make_shared<RobotDrive>(*encoder_)),
      lidar_(std::make_shared<Lidarnode>()), camera_(std::make_shared<CameraControl>()) {
      


 encoder_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

camera_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        

  drive_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);




encoder_timer_ = this->create_wall_tmer(
    std::chrono::milliseconds(10),
    [this]() {
        if(!rclcpp::ok()) return;

        encoder_->callback();
    },
    encoder_group_);



    sensor_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() { 
        if (rclcpp::ok()) {

        if(!rclcpp::ok()) return;
        static const auto start_time = this->now();
        
        lidar_->timer_callback();

        if ((this->now() - start_time).seconds() >= 3.0) {
            lidar_->stopLidar();
            sensor_timer_->cancel();
        }
    }
     }, 
    encoder_group_);

   drive_timer_ = this->create_wall_timer(

            std::chrono::milliseconds(20),

            [this]() { 
              if(rclcpp::ok()) { 
             if(!rclcpp::ok()) return;
                
                drive_->callback(); }
                },
            drive_group_

        );

    }
        rclcpp::SubscriptionOptions options;
        options.callback_group = camera_group_;

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_raw",
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr msg)
        { if(camera->onOroff)
          camera_->getcolor(msg); 
        },
    encoder_group_);

   
private:
  rclcpp::CallbackGroup::SharedPtr encoder_group_;
  rclcpp::CallbackGroup::SharedPtr drive_group_;
    std::shared_ptr<Encoderma> encoder_;
    std::shared_ptr<RobotDrive> drive_;
    std::shared_ptr<Lidarnode> lidar_;
    std::shared_ptr<CameraControl> camera_;
    rclcpp::TimerBase::SharedPtr encoder_timer_;
    rclcpp::TimerBase::SharedPtr drive_timer_;
    rclcpp::TimerBase::SharedPtr sensor_timer_;
     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; //realsenseにくるtopic "camera/camera/color/image_raw"の中のデータをを受け取るため
};
