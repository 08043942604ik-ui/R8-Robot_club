#pragma once

#include <rclcpp/rclcpp.hpp>
#include "color_senser.h"
#include "robotdrive.h"
#include "Encoder.h"
#include <iostream>
#include "Lidar.h"
#include <atomic>
#include "General.h"


class Robotmain : public rclcpp::Node {
public:
    Robotmain(std::shared_ptr<Encoderma> encoder_1, std::shared_ptr<Generalva> genele_)
        : Node("robot_control"),
    encoder_(encoder_1),
      drive_(std::make_shared<RobotDrive>(*encoder_1)),
      lidar_(std::make_shared<Lidarnode>()), color_senser_(std::make_shared<CameraControl>(encoder_)), gee(genele_) {




 encoder_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

 drive_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
 
camera_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

sensor_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

encoder_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() {
        if(stopping_.load())
    RCLCPP_INFO(this->get_logger(), "stopping is true");
              
         if (stopping_.load()) {
             return;
 }
 
        encoder_->callback();
    },
    encoder_group_);



    sensor_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() { 
        
        if (rclcpp::ok()) {

              
        if (stopping_.load()) {
           return;
  }
        static const auto start_time = this->now();
        
        lidar_->timer_callback();

        if ((this->now() - start_time).seconds() >= 3.0) {
            lidar_->stopLidar();
            sensor_timer_->cancel();
        }
    }
     }, 
    sensor_group_);

   drive_timer_ = this->create_wall_timer(

            std::chrono::milliseconds(50),

            [this]() { 
           
                if (drive_->Check_yourself()) {
                 return;
               }
              if(rclcpp::ok()) { 
                  
              if (stopping_.load()) {
                   return;
        }
               
                drive_->callback(); }
                },
            drive_group_

        );

   
               rclcpp::SubscriptionOptions options;
        options.callback_group = camera_group_;

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw",
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr msg)
        { 
    
            color_senser_->getcolor(msg);
}
, options);      
   
   
    }

~Robotmain()
{

request_stop();

    drive_.reset();
    encoder_.reset();
    lidar_.reset();
    color_senser_.reset();

}


void request_stop()
{
     if (stopping_.exchange(true)) {
        return;
    }
 

   drive_->Exchanger(true);



    if (encoder_timer_) encoder_timer_->cancel();
    if (drive_timer_) drive_timer_->cancel();
    if (sensor_timer_) sensor_timer_->cancel();
    
    gee->LWaitingThread();

    if (encoder_) {
        encoder_->StopAll();
    }
    
    
}
   

private:
  rclcpp::CallbackGroup::SharedPtr encoder_group_;
  rclcpp::CallbackGroup::SharedPtr drive_group_;
  rclcpp::CallbackGroup::SharedPtr camera_group_;
    std::shared_ptr<Encoderma> encoder_;
    std::shared_ptr<RobotDrive> drive_;
    std::shared_ptr<Lidarnode> lidar_;
    std::shared_ptr<CameraControl> color_senser_;
     rclcpp::TimerBase::SharedPtr encoder_timer_;
     rclcpp::TimerBase::SharedPtr drive_timer_;
     rclcpp::TimerBase::SharedPtr sensor_timer_;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; //realsenseにくるtopic "camera/camera/color/image_raw"の中のデータをを受け取るため
     std::atomic_bool stopping_{false};
     rclcpp::CallbackGroup::SharedPtr sensor_group_;
     std::shared_ptr<Generalva> gee;

    };