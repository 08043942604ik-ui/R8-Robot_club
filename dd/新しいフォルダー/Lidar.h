#pragma once


#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <memory>
#include "core/common/ydlidar_datatype.h" //CYdlidar.h側でVMXPi.hとぶつかるからLaserScanを使うためにこれがあるヘッダを直接インクルードする

class Lidarinit;

class Lidarnode : public rclcpp::Node {
public:
    Lidarnode();
    ~Lidarnode();

    void timer_callback();
    std::vector<float> disGetter();
    std::vector<float> SendDBW(const std::vector<float>& k);
    void stopLidar();
    double getDistanceAtDegree(double target_deg);

private:
    rclcpp::TimerBase::SharedPtr timer_; 
    std::vector<float> distanceByWalls;
    std::unique_ptr<Lidarinit> lidars_;
    LaserScan raw_data_;
    bool dud = true;
    bool once = true;
};
