#include "Lidar.h"
#include "lidartest.h"


Lidarnode::Lidarnode()
    : Node("lidar"), lidars_(std::make_unique<Lidarinit>())//Rdrivの中がnullptrになるから
{
 RCLCPP_INFO(get_logger(), "Lidar constructor");

    const bool init_ok = lidars_->initi();

    RCLCPP_INFO(
        get_logger(),
        "Lidar initialize = %s",
        init_ok ? "OK" : "FAILED");

    if (!init_ok) {
        return;
    }

}

Lidarnode::~Lidarnode()
{   
    RCLCPP_INFO(get_logger(), "Lidar destructor called");
    lidars_->End();
    RCLCPP_INFO(get_logger(), "Lidar End finished");

    
}

void Lidarnode::stopLidar()
{
    lidars_->End();
}

double Lidarnode::getDistanceAtDegree(double target_deg)
{
   if (raw_data_.points.empty()) {
        return -1.0;
    }

    const double target_rad = target_deg * M_PI / 180.0;//目的の角度をラジアンに変換

    const LaserPoint* nearest = &raw_data_.points[0];//0番目の点が一番近いと仮定

    for (const LaserPoint& point : raw_data_.points) {
        if (std::abs(point.angle - target_rad) <
            std::abs(nearest->angle - target_rad)) {
            nearest = &point;//一番近いpointのアドレスを保管
        }
    }

    return nearest->range;
}

std::vector<float> Lidarnode::disGetter()
{
   if (lidars_->getScan(raw_data_)) {
        distanceByWalls.clear();

        for (const auto& point : raw_data_.points) {
            distanceByWalls.push_back(point.range);
        }
    }

    return distanceByWalls;
}


std::vector<float> Lidarnode::SendDBW(
    const std::vector<float>& k)
{
    return k;
}





void Lidarnode::timer_callback()
{
  if(!rclcpp::ok()) return;

  if(once) {
  lidars_->start();
  
  once = false;
  }

    if (!lidars_->isScanning()) {
        
        return;
    }
if(dud) {
  
    const auto distances = disGetter();

  RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Lidar points=%zu first=%.3f m",
            distanceByWalls.size(),
            distanceByWalls.empty() ? -1.0f : distanceByWalls[0]);
      double d = getDistanceAtDegree(0.0);



  RCLCPP_INFO(get_logger(), "running dd, 0°の壁との距離%lf", d);
    
  SendDBW(distances);
    }


}


