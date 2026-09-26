#include <cmath>
#include "robotdrive.h"
#include <stdexcept>

namespace
{
constexpr double kPi = 3.14159265358979323846;


}


RobotDrive::RobotDrive(Encoderma& encoder)
    : Node("robot_drive"), encoders_(encoder), genel_(std::make_shared<Generalva>()) {
    Xof[0] = 0.0;
    Xof[1] = 0.0;
    s = 0.0;
    x = 0.0;
    y = 0.0;
    Cx = 0.0;
    Cy = 0.0;
  turn_count = 0.0;  
  once = true;        
  deg = 0;             
  hypotenuse = 0.0;  
  DFH = 0.0;         
  AbsDegree = 0;       
  turndeg = 0;          
  result = 0;          
  add = 0;              
  timer = true; 
  w = 0;
  s = 0.0;
  ll = 0.0;
  pp = 0.0;
  checker = 0; 
  k = 0.0;
  po = 0;

encoders_.StopAll();
}


RobotDrive::~RobotDrive() {

encoders_.StopAll();
if (genel_->Turna_thread.joinable()) {
  genel_->Turna_thread.join();
}
}



    
int RobotDrive::GetDegree_man()
{
  if(!rclcpp::ok()) return 0;  
  
  if(Mrun) {
   
      double dou = wastedist.load();

     turn_count = dou - hole;


    Kakudo2 = GetDegree(turn_count);


    hole = wastedist.load();

if(RLC && Kakudo2 == 90) {
Kakudo2 = 90;
RLC = false;
}

if(RLC && Kakudo2 == 180) {
Kakudo2 = 180;
RLC = false;
}      

if(RLC && Kakudo2 == 270) {
Kakudo2 = 270;
RLC = false;
}
Kakudo1.store(Kakudo1.load() + Kakudo2);

while (Kakudo1.load() >= 360) {
      Kakudo1.store(Kakudo1.load() - 360);
    }
      
while (Kakudo1.load() <= -360) {
      Kakudo1.store(Kakudo1.load() + 360);
    }
    
Mrun = false;
}


  return Kakudo1;
  
} //GetDegree_man,END


int RobotDrive::GetDegree(double turnDistance)
{
  if(!rclcpp::ok()) return 0;

  add = AverageChecker();
  once = true;

 
po = add * 90;
    
if(encoders_.left) {
po = po * -1;
}
    
encoders_.right = false;
encoders_.left = false;
return po;
}//GetDegree,END


std::array<double, 2> RobotDrive::IncreaseAxis(int degrees)
{


 if(!rclcpp::ok()) return {0.0, 0.0};
 
  if (once) {
     
// 隴悟唱螻馴ｶ・ｴ陟募ｾ個竏ｵ諤呵崕譏ｴ繝ｻ闕ｳﾂ陜玲ｧｭ笆｡邵ｺ螟ｧ貂戊ｲ・ｴ具ｽｽ蜥ｲ・ｽ・ｮ邵ｺ・ｨ陜難ｽｺ雋・・・ｷ譎槫ｱｬ郢ｧ蜑・ｽｿ譎擾ｽｭ繝ｻ  
    deg = degrees;
    ll = wastedist.load();
    
    xyPa = encoders_.GetTotalDistance() - ll;

    Cx = x;
    Cy = y;

    once = false;

  }

  // 陷ｿ・ｳ陜玲ｫ・ｽｻ・｢邵ｺ・ｰ邵ｺ莉｣・定抄・ｿ邵ｺ繝ｻ・ｰ・ｴ陷ｷ蛹ｻ縲堤ｹｧ繧・繝ｻ  // 霑ｴ・ｾ陜ｨ・ｨ髫苓ｲ橸ｽｺ・ｦ邵ｺ・ｫ郢ｧ蛹ｻ・・x/y 邵ｺ・ｨ +/遶上・郢ｧ蜻茨ｽｱ・ｺ郢ｧ竏夲ｽ狗ｸｲ繝ｻ  if (deg == 0) {
  if (deg == 0) {
    xory = true;
    y = Cy + ((encoders_.GetTotalDistance() - ll) - xyPa);
  }
  else if (deg == 90) {
    xory = false;
    x = Cx + ((encoders_.GetTotalDistance() - ll) - xyPa);
  }
  else if (deg == 180) {
    xory = true;
    y = Cy - ((encoders_.GetTotalDistance() - ll) - xyPa);
  }
  else if (deg == 270) {
    xory = false;
    x = Cx - ((encoders_.GetTotalDistance() - ll) - xyPa);
  }

  return {x, y};



}//IncreaseAxis,END

int RobotDrive::returnToOrigin(double Xe, double Ye)
{
  // 陷ｴ貅ｽ縺帷ｸｺ・ｾ邵ｺ・ｧ邵ｺ・ｮ騾ｶ・ｴ驍ｱ螟奇ｽｷ譎槫ｱｬ
  hypotenuse = std::sqrt((Xe * Xe) + (Ye * Ye));
  DFH = std::atan2(-Ye, -Xe) * 180.0 / kPi;
  AbsDegree = static_cast<int>(std::round(DFH));

  // atan2邵ｺ・ｮ驍ｨ蜈域｣｡邵ｺ・ｯ郢晢ｽｩ郢ｧ・ｸ郢ｧ・｢郢晢ｽｳ邵ｺ・ｪ邵ｺ・ｮ邵ｺ・ｧ邵ｲ竏晢ｽｺ・ｦ邵ｺ・ｸ陞溽判驪､邵ｺ蜷ｶ・欺n  DFH = std::atan2(Ye, Xe) * 180.0 / M_PI;

  // 隰ｨ・ｴ隰ｨ・ｰ邵ｺ・ｮ髫苓ｲ橸ｽｺ・ｦ邵ｺ・ｫ闕ｳ・ｸ郢ｧ竏夲ｽ欺n  AbsDegree = static_cast<int>(std::round(DFH));

  // 邵ｺ阮呻ｼ・ｸｺ・ｯ陟募ｾ後定惷貅ｽ縺幄包ｽｩ陝ｶ・ｰ邵ｺ・ｮ陜玲ｫ・ｽｻ・｢郢晢ｽｫ郢晢ｽｼ郢晢ｽｫ郢ｧ螳夲ｽｩ・ｰ郢ｧ竏夲ｽ玖撻・ｴ隰・
  turndeg = 180 - (90 + AbsDegree);

  if (Xe > 0) {
    result = 360 - turndeg;
  }
  else {
    result = turndeg;
  }

  // 0繝ｻ繝ｻ59陟趣ｽｦ邵ｺ・ｫ騾ｶ・ｴ邵ｺ繝ｻ  
  while (result < 0) {
    result += 360;
  }

  while (result >= 360) {
    result -= 360;
  }

  return result;
}//returnToOrigin,END


int RobotDrive::AverageChecker()
{
pp = turn_count;

  for (int g = 1; g < 100; g++) {
    int dd = static_cast<int>(pp);

    int pi = dd / g;
    

  
    if (pi >= 160 && pi <= 180) {
      static_cast<int>(g);
      checker = g;
      return g;
    }
  }


  return 0;
}//AverageChecker,END

bool RobotDrive::RobotMovement(double xe, double ye) {
 
if(!rclcpp::ok()) return false;
 

if(xe >= 0 && xe < 50) {



if(ye >= 0.0 && ye < 50.0) {
if(times == 10) {
if(turnrunning == false) {
encoders_.setspeedstop();

line = encoders_.GetTotalDistance() + 340.0;

turnrunning = true;
int current_w = times;
times = -1;

if(genel_->Turna_thread.joinable()) {
genel_->Turna_thread.join();
}

genel_->Turna_thread = std::thread([this,current_w]() { RobotTurn(current_w); });

std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
}   
}
}


else if(ye >= 600.0 && ye < 650.0) {

if(times == 0 || times == 9) {
  
if(turnrunning == false) {

 encoders_.setspeedstop();

line = encoders_.GetTotalDistance() + 170.0;

turnrunning = true;
int current_w = times;
times = -1;

  
if(genel_->Turna_thread.joinable()) {
        genel_->Turna_thread.join(); 
    }

genel_->Turna_thread = std::thread([this, current_w]() {
RobotTurn(current_w);
    });

std::this_thread::sleep_for(std::chrono::milliseconds(40));

}
}
}

}//x>0



else if(xe >= 600.0 && xe < 650.0) {



if(ye >= 600.0 && ye < 650.0) {
if(times == 1 || times == 8) {
if(turnrunning == false) {

 encoders_.setspeedstop();

line = encoders_.GetTotalDistance() + 170.0;

turnrunning = true;
     int current_w = times; 
      times = -1; 

if(genel_->Turna_thread.joinable()) {
          genel_->Turna_thread.join();
      }

      genel_->Turna_thread = std::thread([this, current_w]() {
          RobotTurn(current_w);
      });

  


std::this_thread::sleep_for(std::chrono::milliseconds(40));
    

}

}

} //600 < y < 650
else if(ye >= 1200.0 && ye < 1250.0) {


if(times == 2 || times == 7) {
if(turnrunning == false) {

 encoders_.setspeedstop();

line = encoders_.GetTotalDistance() + 170.0;

turnrunning = true;
int current_w = times;
times = -1;

  
if(genel_->Turna_thread.joinable()) {
        genel_->Turna_thread.join(); 
    }

genel_->Turna_thread = std::thread([this, current_w]() {
RobotTurn(current_w);
    });

std::this_thread::sleep_for(std::chrono::milliseconds(40));
    
}


}


    
} // 1500 < y <1550
} // 600 < x < 650

else if(xe >= 900.0 && xe < 950.0) {

if(ye >= 1500.0 && ye < 1550.0) {
if(times == 3 || times == 6) {
if(turnrunning == false) {

 encoders_.setspeedstop();

line = encoders_.GetTotalDistance() + 170.0;

turnrunning = true;
int current_w = times;
times = -1;

  
if(genel_->Turna_thread.joinable()) {
        genel_->Turna_thread.join(); 
    }

genel_->Turna_thread = std::thread([this, current_w]() {
RobotTurn(current_w);
    });

std::this_thread::sleep_for(std::chrono::milliseconds(40));

}

}

} // 900 < y < 950
else if(ye >= 2100.0 && ye < 2150.0){


if(times == 4 || times == 5) {
if(turnrunning == false) {

 encoders_.setspeedstop();

line = encoders_.GetTotalDistance() + 340.0;

turnrunning = true;
int current_w = times;
times = -1;

  
if(genel_->Turna_thread.joinable()) {
        genel_->Turna_thread.join(); 
    }

genel_->Turna_thread = std::thread([this, current_w]() {
RobotTurn(current_w);
    });

std::this_thread::sleep_for(std::chrono::milliseconds(40));

}


}


    
} // 1500 < y <1550




}// 600 < x < 650

return true;//murn邵ｺ・ｫ陷茨ｽ･郢ｧ蠕鯉ｽ欺n}

}

double RobotDrive::RobotTurn(int o) {
  
  if(!rclcpp::ok()) return 0.0;

  float d = 0;
  double result_dist = 0.0;
    
//豼ｶ・ｲ邵ｺ・ｫ郢ｧ蛹ｻ笆ｲ邵ｺ・ｦ陞溷ｳｨ・冗ｹｧ荵昶ｲ關謎ｹ昶斡邵ｺ・ｰ隘搾ｽ､
switch(o) {
case 0:
case 2:
case 5:
case 7:
{

float d = 0; //闕ｳﾂ陜玲ｧｭ笆｡邵ｺ繝ｻ 
    while(true) {
      if(!rclcpp::ok() || tum.load()) return 0.0;

      
    if(d != 0.5f) {
   std::this_thread::sleep_for(std::chrono::milliseconds(1));
    encoders_.RightTurn();
    turn_start = encoders_.GetTotalDistance();
    d = 0.5f;
    
}   

    if(encoders_.GetTotalDistance() >= line) {

     encoders_.setspeedstop();
   
     turn_end = encoders_.GetTotalDistance();
     actual_turn_dist = turn_end - turn_start;

    break;
    
}

std::this_thread::sleep_for(std::chrono::milliseconds(1));
}//while,end
}
break;

case 1:    
case 3:
case 6:
case 8:
{

float d = 0; //闕ｳﾂ陜玲ｧｭ笆｡邵ｺ繝ｻ 
    while(true) {
      if(!rclcpp::ok() || tum.load()) return 0.0;

      
    if(d != 0.5f) {
   std::this_thread::sleep_for(std::chrono::milliseconds(1));
    encoders_.LeftTurn();
    turn_start = encoders_.GetTotalDistance();
    d = 0.5f;
    
}   


    if(encoders_.GetTotalDistance() >= line) {

     encoders_.setspeedstop();
     std::this_thread::sleep_for(std::chrono::milliseconds(50));
  turn_end = encoders_.GetTotalDistance();
  actual_turn_dist = turn_end - turn_start;

  
    break;
    
}

std::this_thread::sleep_for(std::chrono::milliseconds(1));
}//while,end
}
break;    
    
case 4:
case 10:
{
 
float d = 0;

    while(true) {     
    if(!rclcpp::ok() || tum.load()) return 0.0;

      if(d != 0.5f) {
 std::this_thread::sleep_for(std::chrono::milliseconds(1));
    encoders_.RightTurn();
   turn_start = encoders_.GetTotalDistance();
    d = 0.5f;
   
}
 
    
    if(encoders_.GetTotalDistance() >= line) {

      encoders_.setspeedstop();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
          turn_end = encoders_.GetTotalDistance();
          actual_turn_dist = turn_end - turn_start;

    break;
}

std::this_thread::sleep_for(std::chrono::milliseconds(1));
}//while,end
}
break;




default :
  break;

}//switch驍ｨ繧・ｽ冗ｹｧ繝ｻ
set = 0;
wastedist.store(wastedist.load() + actual_turn_dist); 

if (o == 0) { times = 1;

}
else if (o == 1) { times = 2;
RLC = true;
}
else if (o == 2) { times = 3;

}
else if (o == 3) { times = 4;
RLC = true;
}
else if (o == 4) { times = 5; 

}
else if (o == 5) { times = 6; 

} 
else if (o == 6) { times = 7; 
RLC = true;
}
else if (o == 7) { times = 8; 

} 
else if (o == 8) { times = 9; 
RLC = true;
}
else if (o == 9) { times = 10; 

}
  
Mrun = true;
turnrunning = false; 

return actual_turn_dist;


}//RobotTurn驍ｨ繧・ｽ冗ｹｧ繝ｻ

void RobotDrive::Exchanger(bool de) {

tum.store(de);
}

void RobotDrive::callback() {

try {
if(!rclcpp::ok()) return;
    


if(set == 0){
std::this_thread::sleep_for(std::chrono::milliseconds(1));
encoders_.setSpeed();



std::this_thread::sleep_for(std::chrono::milliseconds(10));

set = 1;
}

if(begin == false) {
    
if(turnrunning == false && rclcpp::ok()) {     
Marn = RobotMovement(Xof[0], Xof[1]);

delivery = GetDegree_man();

Xof = IncreaseAxis(delivery);


}
}
RCLCPP_INFO(get_logger(), "x座標%.2f : y座標%.2f : total %lf : wast %lf", Xof[0], Xof[1], encoders_.GetTotalDistance(), wastedist.load());
if(!rclcpp::ok()) {

genel_->LWaitingThread();

}

}
catch(const std::exception& errorD) {

RCLCPP_ERROR(this->get_logger(), "エラーが起きました%s", errorD.what());
}
}
