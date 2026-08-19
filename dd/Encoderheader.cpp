#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <titan.h>
#include <VMXPi.h>
#include "General.h"

class Lidarnode;

class Encoderma : public rclcpp::Node {
 public:
  Encoderma();
  ~Encoderma() override;

  
  std::array<double, 2> RLDistance();


  double GetTotalDistance() const;
  void setSpeed();
void setspeedturn();
void setspeedstop();  

  void StopAll();
  void stopAll();

  // 20ms邵ｺ譁絶・邵ｺ・ｫ陷ｻ・ｼ邵ｺ・ｰ郢ｧ蠕個繝ｻ0驕倩ｲ橸ｽｾ蠕後・陋帶㊧・ｭ・｢邵ｺ・ｨ陋滂ｽ､邵ｺ・ｮ髯ｦ・ｨ驕会ｽｺ郢ｧ螳夲ｽ｡蠕娯鴬
  void callback();

 private:
  std::shared_ptr<VMXPi> vmx_;
  std::unique_ptr<studica_driver::Titan> titan_;
  rclcpp::TimerBase::SharedPtr timer_;
   std::shared_ptr<Generalva> general_;
std::array<double, 2> previous_distance_{0.0, 0.0};
  bool first_callback_ = true;
  double right_delta = 0.0;
  double TotalDistance = 0.0;
  double left_delta = 0.0;
};

