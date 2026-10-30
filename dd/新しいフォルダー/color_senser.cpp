#include "color_senser.h"





    CameraControl::CameraControl(std::shared_ptr<Encoderma> encoder)
    : Node("camera_control"), encoder_(encoder)
    { 
     
        publisher_s = create_publisher<std_msgs::msg::Float64>("/flag_value", 10);
      // RealSenseの画像を受け取るSubscriber

        // RealSenseノードのパラメータを操作するClient
        parameter_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(
        this,
        "/realsense_color_node");

    cameraOn();

        RCLCPP_INFO(this->get_logger(), "camera_control started");
    }

void CameraControl::cameraOn()
{
    RCLCPP_INFO(get_logger(), "cameraOn entered");
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

    void CameraControl::cameraOff()
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


void CameraControl::getcolor(const sensor_msgs::msg::Image::ConstSharedPtr msginF)   {
// ROS画像 → OpenCV BGR画像
          
if(!runnign) {
           cv::Mat image =
                cv_bridge::toCvCopy(
                    msginF,
                    sensor_msgs::image_encodings::BGR8)
                    ->image;

            // 画像中央
 int center_x = image.cols / 2;
 int center_y = image.rows / 2;

int sum_r = 0;
int sum_g = 0;
int sum_b = 0;
int count = 0;
int Redcount = 0;
int gr_count = 0;
int nothing = 0;
runnign = true;
for (int y = center_y - 20; y < center_y + 20; y++)
{
    for (int x = center_x - 20; x < center_x + 20; x++)
    {
        cv::Vec3b pixel = image.at<cv::Vec3b>(y, x);

        sum_b = pixel[0];
        sum_g = pixel[1];
        sum_r = pixel[2];

        if(sum_r >= 140 && (sum_r - sum_g) >= 90 && (sum_r - sum_b) >= 60 && (sum_b - sum_g) >= 20) {

        Redcount++;
        }
       
        else if(sum_g >= 120 && (sum_g - sum_r) >= 60 && (sum_g - sum_b) >= 65)
        {
  
            gr_count++;
        }

        else {

         count++;
        }
    
     //ifの判断にservo_nodeからの信号で判断も入れる


    }
}
if(HMtimes == 0) {
OnceRedratio = Redcount / (Redcount + count);

HMtimes++;
}

if(HMtimes == 1) {
twiceRedratio = Redcount / (Redcount + count);

if((OnceRedratio - twiceRedratio) < 0.0) {

RCLCPP_INFO(this->get_logger(), "Second is bigger than first");
}

else if(OnceRedratio == 0.0 && twiceRedratio == 0.0) {

nothing = 1;    
}

else {
RCLCPP_INFO(this->get_logger(), "First is bigger than Second");

}

OnceRedratio = 0.0;
twiceRedratio = 0.0;

std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

runnign = false;
}

}

void CameraControl::cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    getcolor(msg);

RCLCPP_INFO(this->get_logger(), "in the callback");

}
