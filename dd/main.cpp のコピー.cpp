#include "Robotmainmove.h"
#include <stdexcept>
#include <iostream>
#include <General.h>

int main(int argc, char * argv[]) {
    
try {    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Robotmain>();
    auto ganel = std::make_shared<Generalva>(); 

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::cout << "test" << std::endl;
     std::this_thread::sleep_for(std::chrono::milliseconds(1));
    executor.spin();
    std::cout << "test" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
   ganel->LWaitingThread();
    rclcpp::shutdown();
    return 0;
}
 catch(const std::exception& e) {

std::cout << "エラーが発生しました。" << std::endl;


if (rclcpp::ok()) {
    rclcpp::shutdown();
}//イニシャライズしていないとROS2の終了処理に繋げられないから
return 1;
} 

}
