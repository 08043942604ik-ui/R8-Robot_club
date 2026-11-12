#include "servo.h" 
#include <chrono>

servom::servom()
    : Node("servo_node")
{
           publisher_ = create_publisher<std_msgs::msg::Bool>("/GetOrder/Place", 10);
  Stimer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&servom::callback, this));
}

 servom::~servom() {} 
 
 
 
 void servom::callback() { 
 
while(doe == 1) {
std::cout << "Enter angle (0 to 360): " << std::endl;


    std::cin >> angle;
if(angle < -150 || angle > 150) {
    do {
        std::cout << "Invalid input. Please enter an angle between -180 and 0." << std::endl;
        std::cin >> angle;
    } while (angle < -150 || angle > 150);
}
        if (std::cin.fail()) {
            std::cout << "エラー: 数値以外が入力されました。入力し直してください" << std::endl;
            
            std::cin.clear();  
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); 
            continue;          
        }

servo3_->SetAngle(angle);

std::this_thread::sleep_for(std::chrono::milliseconds(2000));



std::cout << "Enter angle (0 to 360): " << std::endl;
        std::cin >> angle3;

if(angle3 < -150 || angle3 > 150) {
    do {
        std::cout << "Invalid input. Please enter an angle between -150 and 150." << std::endl;
        std::cin >> angle3;
    } while (angle3 < -150 || angle3 > 150);
}
        if (std::cin.fail()) {
            std::cout << "エラー: 数値以外が入力されました。入力し直してください" << std::endl;
            
            std::cin.clear();  
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); 
            continue;          
        }    
        
servo_->SetAngle(angle3);

std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::cin >> angle2;
if(angle2 < -150 || angle2 > 150) {
    do {
        std::cout << "Invalid input. Please enter an angle between -180 and 0." << std::endl;
        std::cin >> angle;
    } while (angle2 < -150 || angle2 > 150);
}
        if (std::cin.fail()) {
            std::cout << "エラー: 数値以外が入力されました。入力し直してください" << std::endl;
            
            std::cin.clear();  
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); 
            continue;          
        }

servo2_->SetAngle(angle2);

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
