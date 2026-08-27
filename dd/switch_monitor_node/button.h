#include <studica_drivers/dio.h>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>

class BFE : public rclcpp::Node {
public :

BFE() : Node("button_monitor"), 
        vmx_(std::make_shared<VMXPi>(true, 50)),
        start_button_(11, studica_driver::PinMode::INPUT, vmx_),
        stop_button_(10, studica_driver::PinMode::INPUT, vmx_)
{

   timer_ = create_wall_timer(
   std::chrono::milliseconds(10),
   [this] () {

   check_buttons();

   }

   );


}

private:
void check_buttons()
{
    bool start_now = start_button_.Get();
    bool stop_now = stop_button_.Get();

    if (prev_start_ && !start_now)
    {
        std::system("sudo systemctl start robot_run.service");
    }

    if (prev_stop_ && !stop_now)
    {
        std::system("sudo systemctl stop robot_run.service");
    }

    prev_start_ = start_now;
    prev_stop_ = stop_now;
}

bool prev_start_;
bool prev_stop_;
rclcpp::TimerBase::SharedPtr timer_;
std::shared_ptr<VMXPi> vmx_;
studica_driver::DIO start_button_;
studica_driver::DIO stop_button_;

}; 