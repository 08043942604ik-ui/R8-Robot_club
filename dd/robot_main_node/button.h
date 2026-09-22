#pragma once
#include <studica_drivers/dio.h>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include "accounting.h"

class BFE : public rclcpp::Node {
public :

BFE(std::shared_ptr<VMXPi> vmx_) : Node("button_monitor"),
        start_button_(11, studica_driver::PinMode::INPUT, vmx_),
        stop_button_(10, studica_driver::PinMode::INPUT, vmx_)
{

prev_start_ = start_button_.Get();
prev_stop_ = stop_button_.Get();

   timer_ = create_wall_timer(
   std::chrono::milliseconds(10),
   [this] () {

   check_buttons();

   }

   );


}
bool reset_button_states()
{
    start_button_pressed_ = false;
    stop_button_pressed_ = false;
    return true;
}

bool get_start_button_state()
{
   return start_button_pressed_;
}

bool get_stop_button_state()
{
   return stop_button_pressed_;
}

private:
void check_buttons()
{
    bool start_now = start_button_.Get();
    bool stop_now = stop_button_.Get();

    if (prev_start_ && !start_now)
    {

        start_button_pressed_ = true;

    }

    if (prev_stop_ && !stop_now)
    {
        stop_button_pressed_ = true;
    }

    prev_start_ = start_now;
    prev_stop_ = stop_now;


}

bool prev_start_ = true;
bool prev_stop_ = true;
bool start_button_pressed_ = false;
bool stop_button_pressed_ = false;
rclcpp::TimerBase::SharedPtr timer_;
studica_driver::DIO start_button_;
studica_driver::DIO stop_button_;

}; 
