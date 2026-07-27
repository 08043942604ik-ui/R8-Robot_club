#include "Encoder.h"

#include <chrono>
#include <cmath>

namespace {

constexpr int kCanId = 42;
constexpr int kMotorFrequency = 15600;

constexpr int kRightMotor = 2;  // 右タイヤはM2
constexpr int kLeftMotor = 3;   // 左タイヤはM3

// 5%の低速で60秒間回す
constexpr double kDriveSpeed = 0.05;
constexpr double kDriveSeconds = 60.0;

// 実機のTitanエンコーダー設定
constexpr int kTicksPerRotation = 1470;

// タイヤ直径100mmとして、1回転の距離は約0.314m
constexpr double kWheelCircumferenceM = 0.314159;
constexpr double kDistancePerTickM =
    kWheelCircumferenceM / kTicksPerRotation;

}  // namespace

Encoderma::Encoderma() {
  // VMXを50Hzで初期化
  vmx_ = std::make_shared<VMXPi>(true, 50);
  if (!vmx_->IsOpen()) {
    throw std::runtime_error(
        "VMXを開けません。sudoで実行し、他のVMXプログラムを停止してください。");
  }

  // Titan Quadを初期化
  titan_ = std::make_unique<studica_driver::Titan>(
      static_cast<uint8_t>(kCanId),
      static_cast<uint16_t>(kMotorFrequency),
      static_cast<float>(kTicksPerRotation),
      vmx_);
  titan_->Enable(true);

  // 配線されているENC端子を確認するため、M0～M3を有効化
  for (int motor = 0; motor < 4; ++motor) {
    titan_->SetupEncoder(static_cast<uint8_t>(motor));
    titan_->ConfigureEncoder(static_cast<uint8_t>(motor), kDistancePerTickM);
    titan_->ResetEncoder(static_cast<uint8_t>(motor));
  }

  StopAll();

  // 20msごとにcallback()を実行
  timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() { callback(); });
}

Encoderma::~Encoderma() {
  StopAll();
}
void Encoderma::stopAll() {
  for (int motor = 0; motor < 4; ++motor) {
    titan_->SetSpeed(static_cast<uint8_t>(motor), 0.0);
  }
}

void Encoderma::setSpeed(uint8_t motor, double speed) {
    titan_->SetSpeed(motor, speed);
}//違うファイルで使うよう

std::array<double, 2> Encoderma::RLDistance() {
  return {titan_->GetEncoderDistance(static_cast<uint8_t>(kLeftMotor)), titan_->GetEncoderDistance(static_cast<uint8_t>(kRightMotor))};
}


void Encoderma::StopAll() {
  if (!titan_) {
    return;
  }

  // 安全のためTitanのM0～M3をすべて停止
  for (int motor = 0; motor < 4; ++motor) {
    titan_->SetSpeed(static_cast<uint8_t>(motor), 0.0);
  }
}


void Encoderma::callback() {


auto distance = RLDistance();


  // 前回から動いた距離を左右で平均し、総移動距離に加算
   double left_delta = std::abs(distance[0]);
   double right_delta = std::abs(distance[1]);
  TotalDistance += (left_delta + right_delta) / 2.0;

  RCLCPP_INFO(get_logger(), "左:%.3fm 右:%.3fm 合計:%.3fm", distance[0], distance[1], TotalDistance);
}
