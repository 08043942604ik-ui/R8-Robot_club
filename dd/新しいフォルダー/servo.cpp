#include "servo.h" 
#include <chrono>

servom::servom()
    : Node("servo_node"),
      servo_(std::make_shared<studica_driver::Servo>(
          21,
          studica_driver::ServoType::Standard))
{
           publisher_ = create_publisher<std_msgs::msg::Bool>("/GetOrder/Place", 10);
  
}

 servom::~servom() {} 
 
 
 
 void servom::callback() { 
 
while(doe == 1) {
std::cout << "Enter angle (-90 to 0): " << std::endl;


    std::cin >> angle;
if(angle < -180 || angle > 0) {
    do {
        std::cout << "Invalid input. Please enter an angle between -180 and 0." << std::endl;
        std::cin >> angle;
    } while (angle < -180 || angle > 0);
}
        if (std::cin.fail()) {
            std::cout << "エラー: 数値以外が入力されました。入力し直してください" << std::endl;
            
            std::cin.clear();  
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); 
            continue;          
        }

servo_->SetAngle(angle);
std::this_thread::sleep_for(std::chrono::milliseconds(2000));
std::cout << "Angle set to: " << angle << std::endl;


std::cout << "Continue? (Yes -> 1/ No -> 0): " << std::endl;
std::cin >> doe;
if(doe != 0 && doe != 1) {
    do {
        std::cout << "Invalid input. Please enter 1 for Yes or 0 for No." << std::endl;
        std::cin >> doe;
    } while (doe != 0 && doe != 1);
}

if(doe == 0) {
    std::cout << "Exiting..." << std::endl;
    break;
}


std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
}