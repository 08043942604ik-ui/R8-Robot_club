#include "color_senser.h"





    CameraControl::CameraControl()
    : Node("camera_control")
    { 
     
        publisher_s = create_publisher<std_msgs::msg::Float64>("/Take/this", 10);
      // RealSenseの画像を受け取るSubscriber

        // RealSenseノードのパラメータを操作するClient
        parameter_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(
        this,
        "/realsense_color_node");

    subscriber_ = create_subscription<std_msgs::msg::Bool>(
    "/GetOrder/Place", 10,
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        received_flag_ = msg->data;

        if (received_flag_) {
        if (!Ondone) {
            cameraOn();
        }
        }    
         else {
            if(Ondone) {
            cameraOff();
            }
        }
    
    }


    
);

image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw",
    rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cameraCallback(msg);
    }
);


        RCLCPP_INFO(this->get_logger(), "camera_control started");
    }

void CameraControl::cameraOn()
{
    if(!running) {
 
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
           Ondone = true;
    }
}

    void CameraControl::cameraOff()
{
if(!running) {

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
        Ondone = false;
    }
}


void CameraControl::getcolor(const sensor_msgs::msg::Image::ConstSharedPtr msginF)   {
// ROS画像 → OpenCV BGR画像
          
if(!runnign) {
    running = true;
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
for (int y = center_y - 50; y < center_y + 50; y++)
{
    for (int x = center_x - 50; x < center_x + 50; x++)
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
if(!(Redcount == 0) && times < 100) {

OnceRedratio += (double)Redcount / (double)(Redcount + gr_count + count);


times++;
}

if(!(gr_count == 0) && times < 100) {

Oncegreen += (double)gr_count / (double)(Redcount + gr_count + count);


times++;
}

if(times == 100 && OnceRedratio > 0.0) {

OnceRedratio = OnceRedratio / 100.0;
std_msgs::msg::Float64 msg;
msg.data = OnceRedratio + 100000.0; //100000を足すことで赤色の割合が0.0~1.0の範囲であっても、赤色が検出されたことを示すために100000を足す
publisher_s->publish(msg);
times = 0;
OnceRedratio = 0.0;
running = false;    
}


if(times == 100 && Oncegreen > 0.0) {

Oncegreen = Oncegreen / 100.0;
std_msgs::msg::Float64 msg;
msg.data = Oncegreen + 10000.0; //100000を足すことで赤色の割合が0.0~1.0の範囲であっても、赤色が検出されたことを示すために100000を足す
publisher_s->publish(msg);
times = 0;
Oncegreen = 0.0;
running = false;    
}

runnign = false;
}

}

void CameraControl::cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    getcolor(msg);

RCLCPP_INFO(this->get_logger(), "in the callback");

}
