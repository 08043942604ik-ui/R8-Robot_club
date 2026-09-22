#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>

class Robotmain;
class BFE;
class Encoderma;
class Generalva;

class SVE : public rclcpp::Node {
public:
    SVE(std::shared_ptr<Encoderma> encoder, std::shared_ptr<BFE> buttons);
    ~SVE() override;

    void executorAdder(rclcpp::Node::SharedPtr node);
    void executorRemover(rclcpp::Node::SharedPtr node);
    void executorSpin_some();
    bool check_status();

private:
    std::shared_ptr<BFE> buttons_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<Robotmain> robot_run;
    std::shared_ptr<Encoderma> encoder_1;
    std::shared_ptr<Generalva> general_values_;
    bool bo = false;
    bool status = false;
};
