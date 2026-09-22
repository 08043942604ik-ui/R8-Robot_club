#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <titan.h>
#include "General.h"
#include "Encoder.h"

class RobotDrive : public rclcpp::Node {
 public:
  explicit RobotDrive(Encoderma& encoder);
  ~RobotDrive() override;

  int GetDegree_man();
  int GetDegree(double turnDistance);
  std::array<double, 2> IncreaseAxis(int degrees);
  int returnToOrigin(double Xe, double Ye);
  int AverageChecker();
  bool RobotMovement(double xe, double ye);
  double turnDistanceChecker();
  void callback();
 
   bool mos = false; //lidar起動フラグ
 
  private:

  double RobotTurn(int o);
  
  Encoderma& encoders_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<double, 2> Xof{0.0, 0.0};
  std::atomic<bool> turnrunning{false};
  std::atomic<double> wastedist{0.0};
  std::atomic<int> times{0};
  std::atomic<int> Kakudo1{0};
  std::atomic<int> Kakudo2{0};
  std::shared_ptr<Generalva> genel_;
   

  double Cx = 0.0;
  double Cy = 0.0;
  double x = 0.0;
  double y = 0.0;
  double d = 0.0;
  double hole = 0.0;
  double xyPa = 0.0;
  double line = 0.0;
  int delivery = 0;
  int set = 0;
  bool begin = false;
  bool xory = false;
  bool Marn = false;
  bool Mrun = false;
  double turn_start = 0.0;
  double turn_end = 0.0;
  double actual_turn_dist = 0.0;
};
