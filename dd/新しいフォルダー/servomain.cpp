#include "servo.h"


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
auto node = std::make_shared<servom>();

node->callback();

return 0;
}