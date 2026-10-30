#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <librealsense2/rs.hpp>

#include <chrono>
#include <cstring>
#include <memory>

class RealSenseColorNode : public rclcpp::Node
{
public:
    RealSenseColorNode();
 
    ~RealSenseColorNode();
    
    void publish_frame();

private:

  

    rs2::pipeline pipeline_;
    bool started_ = false;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic_bool color_enabled_{false};

rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_;
};