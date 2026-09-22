#include "Robotmainmove.h"
#include "button.h"
#include "accounting.h"
#include <exception>
#include <iostream>
#include <General.h>

int main(int argc, char * argv[]) {
    
try {    
    rclcpp::init(argc, argv);
    
    auto vmx = std::make_shared<VMXPi>(true, 50);
    auto encoder = std::make_shared<Encoderma>(vmx);
    auto button_node = std::make_shared<BFE>(vmx);
    auto accounta = std::make_shared<SVE>(encoder, button_node);
   
    accounta->executorAdder(accounta);
    accounta->executorAdder(button_node);
     std::this_thread::sleep_for(std::chrono::milliseconds(1));
     while(rclcpp::ok()) {
    accounta->executorSpin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
   
    }
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
