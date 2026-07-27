"Robotmainmove.h"


int main(int argc, char * argv[]) {
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
