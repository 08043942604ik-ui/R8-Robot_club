#include "Encoder.h"
#include <stdexcept>
#include <chrono>
#include <cmath>
#include <thread>
#include "Lidar.h"

namespace {

constexpr double Rspeed = -0.155;
constexpr double Lspeed = 0.15;

constexpr double RTspeed = -0.15;
constexpr double LTspeed = -0.15;

constexpr int kCanId = 42;
constexpr int kMotorFrequency = 15600;
constexpr double kDriveSpeed = 0.05;
constexpr int kTicksPerRotation = 1470;

constexpr int kRightMotor = 0;  // 陷ｿ・ｳ郢ｧ・ｿ郢ｧ・､郢晢ｽ､邵ｺ・ｯM2
constexpr int kLeftMotor = 1;   // 陝ｾ・ｦ郢ｧ・ｿ郢ｧ・､郢晢ｽ､邵ｺ・ｯM3

// 5%邵ｺ・ｮ闖ｴ譛ｱﾂ貅倥・0驕伜ｸ昜ｿ｣陜玲ｧｭ笘・nconstexpr double kDriveSpeed = 0.05;
constexpr double kDriveSeconds = 60.0;

// 陞ｳ貊難ｽｩ貅倥・Titan郢ｧ・ｨ郢晢ｽｳ郢ｧ・ｳ郢晢ｽｼ郢敖郢晢ｽｼ髫ｪ・ｭ陞ｳ繝ｻconstexpr int kTicksPerRotation = 1470;

// 郢ｧ・ｿ郢ｧ・､郢晢ｽ､騾ｶ・ｴ陟輔・00mm邵ｺ・ｨ邵ｺ蜉ｱ窶ｻ邵ｲ繝ｻ陜玲ｫ・ｽｻ・｢邵ｺ・ｮ髴肴辨螻ｬ邵ｺ・ｯ驍上・.314m
constexpr double kWheelCircumferenceM = 0.314159;
constexpr double kDistancePerTickM =
    kWheelCircumferenceM / kTicksPerRotation;

}  // namespace

Encoderma::Encoderma() : Node("studicaTiEC") {
  TotalDistance = 0.0;
  left_delta = 0.0;
  right_delta = 0.0;

  vmx_ = std::make_shared<VMXPi>(true, 50);
  // VMX郢ｧ繝ｻ0Hz邵ｺ・ｧ陋ｻ譎・ｄ陋ｹ繝ｻ  vmx_ = std::make_shared<VMXPi>(true, 50);
  if (!vmx_->IsOpen()) {
    throw std::runtime_error(
        /*
        "VMX郢ｧ蟶晏ｹ慕ｸｺ莉｣竏ｪ邵ｺ蟶呻ｽ鍋ｸｲ・ｴudo邵ｺ・ｧ陞ｳ貅ｯ・｡蠕鯉ｼ邵ｲ竏ｽ・ｻ謔ｶ繝ｻVMX郢晏干ﾎ溽ｹｧ・ｰ郢晢ｽｩ郢晢｣ｰ郢ｧ雋樞酪雎・ｽ｢邵ｺ蜉ｱ窶ｻ邵ｺ荳岩味邵ｺ霈費ｼ樒ｸｲ繝ｻ);
        */
        /*
        "VMXを開けません。sudoで実行し、他のVMXプログラムを停止してください。");
        */
        "VMX open failed. Run as root and stop other VMX programs.");
  }

  

  // Titan Quad郢ｧ雋槭・隴帶ｺｷ蝟ｧ
  titan_ = std::make_unique<studica_driver::Titan>(
      static_cast<uint8_t>(kCanId),
      static_cast<uint16_t>(kMotorFrequency),
      static_cast<float>(kTicksPerRotation),
      vmx_);
  titan_->Enable(true);

  // 鬩溷調・ｷ螢ｹ・・ｹｧ蠕娯ｻ邵ｺ繝ｻ・畿NC驕ｶ・ｯ陝・・・帝￡・ｺ髫ｱ髦ｪ笘・ｹｧ荵昶螺郢ｧ竏堋・0繝ｻ譛・郢ｧ蜻域剰怏・ｹ陋ｹ繝ｻ  for (int motor = 0; motor < 4; ++motor) {
  for (int motor = 0; motor < 4; ++motor) {
    titan_->SetupEncoder(static_cast<uint8_t>(motor));
    titan_->ConfigureEncoder(static_cast<uint8_t>(motor), kDistancePerTickM);
    titan_->ResetEncoder(static_cast<uint8_t>(motor));
  }

  StopAll();


}

Encoderma::~Encoderma() {
    TotalDistance = 0.0;


  StopAll();
}
void Encoderma::stopAll() {
  for (int motor = 0; motor < 4; ++motor) {
    titan_->SetSpeed(static_cast<uint8_t>(motor), 0.0);
  }
}


void Encoderma::setSpeed() {


    titan_->SetSpeed(static_cast<uint8_t>(kLeftMotor), Lspeed);
    RCLCPP_INFO(this->get_logger(), "モーターに指示が送られた");
    titan_->SetSpeed(static_cast<uint8_t>(kRightMotor), Rspeed);  
}

void Encoderma::setspeedturn() {
    titan_->SetSpeed(static_cast<uint8_t>(kLeftMotor), LTspeed);
       RCLCPP_INFO(this->get_logger(), "モーターに指示が送られた");
    titan_->SetSpeed(static_cast<uint8_t>(kRightMotor), RTspeed);
 }

void Encoderma::setspeedstop() {
    titan_->SetSpeed(static_cast<uint8_t>(kLeftMotor), 0.0);
       RCLCPP_INFO(this->get_logger(), "モーターに指示が送られた");
    titan_->SetSpeed(static_cast<uint8_t>(kRightMotor), 0.0);
 }

std::array<double, 2> Encoderma::RLDistance() {
  return {titan_->GetEncoderDistance(static_cast<uint8_t>(kLeftMotor)), titan_->GetEncoderDistance(static_cast<uint8_t>(kRightMotor))};
}

double Encoderma::GetTotalDistance() const {
  return TotalDistance * 1000.0;
}


void Encoderma::StopAll() {
  if (!titan_) {
    return;
  }

  // 陞ｳ迚吶・邵ｺ・ｮ邵ｺ貅假ｽゝitan邵ｺ・ｮM0繝ｻ譛・郢ｧ蛛ｵ笘・ｸｺ・ｹ邵ｺ・ｦ陋帶㊧・ｭ・｢
  for (int motor = 0; motor < 4; ++motor) {
    titan_->SetSpeed(static_cast<uint8_t>(motor), 0.0);
  }
}


void Encoderma::callback() {

try {



  const int left_count =
    titan_->GetEncoderCount(static_cast<uint8_t>(kLeftMotor));

const int right_count =
    titan_->GetEncoderCount(static_cast<uint8_t>(kRightMotor));

    
  const auto distance = RLDistance();

    if (first_callback_) {
        previous_distance_ = distance;
        first_callback_ = false;
        return;
    }

    left_delta =
        std::abs(distance[0] - previous_distance_[0]);

    right_delta =
        std::abs(distance[1] - previous_distance_[1]);

    TotalDistance += right_delta;

    previous_distance_ = distance;



RCLCPP_INFO(this->get_logger(), "右のエンコーダーカウント%d : 左のエンコーダーカウント%d トータル距離%lf 右%lf, 左%lf", right_count, left_count, TotalDistance, right_delta, left_delta);



}


catch(const std::exception& error) {

RCLCPP_ERROR(this->get_logger(), "郢ｧ・ｨ郢晢ｽｩ郢晢ｽｼ邵ｺ讙主験騾墓ｺ假ｼ邵ｺ・ｾ邵ｺ蜉ｱ笳・%s", error.what());

    
}
}


