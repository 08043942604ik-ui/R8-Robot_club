#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <sharp.h>
#include <studica_drivers/dio.h>
#include "color_senser.h"

class Robotmain;
class BFE;
class Encoderma;
class Generalva;
class VMXPi;

class SVE : public rclcpp::Node {
public:
  SVE(std::shared_ptr<VMXPi> vmx_, std::shared_ptr<BFE> buttons);
    ~SVE() override;

    void executorAdder(rclcpp::Node::SharedPtr node);
    void executorRemover(rclcpp::Node::SharedPtr node);
    void executorSpin_some();
    bool check_status();
    void finalizeRobotStop();
private:
    std::shared_ptr<BFE> buttons_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::shared_ptr<Robotmain> robot_run;
    std::shared_ptr<Encoderma> encoder_1;
    std::shared_ptr<Generalva> general_values_;
    std::shared_ptr<VMXPi> vmx1;
    bool status = false;
    bool pending_robot_stop_ = false;
    std::shared_ptr<studica_driver::Sharp> giver_;
    std::shared_ptr<studica_driver::DIO> dio1_;
    std::shared_ptr<studica_driver::DIO> dio_;
    std::shared_ptr<studica_driver::DIO> dio2_;
    bool ch = false;
    std::shared_ptr<CameraControl> camera_;
  };