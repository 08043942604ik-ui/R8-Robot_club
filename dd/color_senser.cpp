#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp> //realsenseのnodeと通信するため
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class CameraControl : public rclcpp::Node
{
public:
    CameraControl()
    : Node("camera_control"),
      use_image_(true)
    { 
      
      // RealSenseの画像を受け取るSubscriber

        // RealSenseノードのパラメータを操作するClient
        parameter_client_ =
            std::make_shared<rclcpp::AsyncParametersClient>(
                this,
                "/camera/camera");

        RCLCPP_INFO(this->get_logger(), "camera_control started");
    }

void cameraOn()
{
    if (!parameter_client_->wait_for_service(2s))
    {
        RCLCPP_ERROR(get_logger(), "Camera node not found");
        return;
    }

    parameter_client_->set_parameters(
        {rclcpp::Parameter("enable_color", true)}, 
        [this](auto future)
        {
            auto results = future.get();

            if (!results.empty() && results[0].successful)
                RCLCPP_INFO(get_logger(), "Camera ON");
            else
                RCLCPP_ERROR(get_logger(), "Camera ON failed");
        }); //設定した分だけ配列の数は増える今回は1個なのでindexは0
}

    void cameraOff()
{
    if (!parameter_client_->wait_for_service(2s))
    {
        RCLCPP_ERROR(get_logger(), "Camera node not found");
        return;
    }

    parameter_client_->set_parameters(
        {rclcpp::Parameter("enable_color", false)},
        [this](auto future)
        {
            auto results = future.get();

            if (!results.empty() && results[0].successful)
                RCLCPP_INFO(get_logger(), "Camera Off");
            else
                RCLCPP_ERROR(get_logger(), "Camera Off failed");
        });
}
int getcolor(const sensor_msgs::msg::Image::ConstSharedPtr msginF)   {
　　// ROS画像 → OpenCV BGR画像
            cv::Mat image =
                cv_bridge::toCvCopy(
                    msginF,
                    sensor_msgs::image_encodings::BGR8)
                    ->image;

            // 画像中央
            int x = image.cols / 2;
            int y = image.rows / 2;

int sum_r = 0;
int sum_g = 0;
int sum_b = 0;
int count = 0;

for (int y = center_y - 10; y < center_y + 10; y++)
{
    for (int x = center_x - 10; x < center_x + 10; x++)
    {
        cv::Vec3b pixel = image.at<cv::Vec3b>(y, x);

        sum_b += pixel[0];
        sum_g += pixel[1];
        sum_r += pixel[2];

        count++;
    }
}

int avg_r = sum_r / count;
int avg_g = sum_g / count;
int avg_b = sum_b / count;　
          

}

private:
    

    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr image_sub_; //realsenseにくるtopic "camera/camera/color/image_raw"の中のデータをを受け取るため

    std::shared_ptr<
        rclcpp::AsyncParametersClient> parameter_client_; //realsenseノードと通信するため\

bool onOroff;
};

