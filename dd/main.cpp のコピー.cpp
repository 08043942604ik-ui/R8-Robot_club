#include "Robotmainmove.h"
#include <stdexcept>



int main(int argc, char * argv[]) {
    
try {    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Robotmain>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin(); 

if(turn_thread.joinable()) {
    turn_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}
 catch(const std::exception& e) {

rclcpp::init(argc, argv);
RCLCPP_ERROR(rclcpp::get_logger("main"), "エラーが発生しました %s", e.what());
return 1;
}  

}
