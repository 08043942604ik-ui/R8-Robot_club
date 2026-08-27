#include "button.h"
#include <stdexcept>
#include <iostream>

int main(int argc, char ** argv) {
try {
rclcpp::init(argc, argv);

auto Bnode = make_shared<BFE>();

rclcpp::spin(Bnode);

rclcpp::shutdown();

return 0;
}
catch(std::exception& errorm) {

std::cout << "error" << std::endl;

}
}
