#include <cmath>
#include "robotdrive.h"
#include <stdexcept>

namespace
{
constexpr double kPi = 3.14159265358979323846;
double turn_count = 0.0;   // 回転したときの走行距離
bool once = true;         // IncreaseAxis邵ｺ・ｮ隴崢陋ｻ譏ｴ笆｡邵ｺ鬘鯉ｽ｡蠕娯鴬陷・ｽｦ騾・・逡曾nint Kakudo = 0;            // 霑ｴ・ｾ陜ｨ・ｨ邵ｺ・ｮ髫苓ｲ橸ｽｺ・ｦ
int deg = 0;               // IncreaseAxis邵ｺ・ｧ闖ｴ・ｿ邵ｺ繝ｻ・ｧ雋橸ｽｺ・ｦ
double hypotenuse = 0.0;   // 陷ｴ貅ｽ縺帷ｸｺ・ｾ邵ｺ・ｧ邵ｺ・ｮ髴肴辨螻ｬ
double DFH = 0.0;          // atan2邵ｺ・ｧ陷・ｽｺ邵ｺ蜉ｱ笳・囓雋橸ｽｺ・ｦ
int AbsDegree = 0;         // 陜怜ｸｶ譏剰滋豕後・陟募ｾ後・髫苓ｲ橸ｽｺ・ｦ
int turndeg = 0;          // 陷ｴ貅ｽ縺帷ｸｺ・ｸ陷ｷ莉｣・･邵ｺ貅假ｽ∫ｸｺ・ｮ陜玲ｫ・ｽｻ・｢髫苓ｲ橸ｽｺ・ｦ
int result = 0;           // 驍ｨ蜈域｣｡
int add = 0;              // 90陟趣ｽｦ郢ｧ蜑・ｽｽ蜍溷ｱ楢崕繝ｻ螻鍋ｸｺ・｣邵ｺ貅伉ｰ
bool turnrunning = false; //隶匁ｻ会ｽｽ阮吮ｲ陜玲ｫ・ｽｻ・｢闕ｳ・ｭ邵ｺ繝ｻint times = 0;//闖ｴ蜍溷ｱ楢ｮ匁ｻ会ｽｽ阮吮ｲ陜玲ｫ・ｽｻ・｢邵ｺ蜉ｱ窶ｻ郢ｧ荵敖ｰ
bool timer = true; //郢ｧ・ｿ郢ｧ・､郢晄ｧｭ繝ｻ郢ｧ・ｹ郢ｧ・ｿ郢晢ｽｼ郢晏現・定叉ﾂ陜玲ｧｭ竊鍋ｸｺ蜷ｶ・狗ｸｺ貅假ｽ―nbool end = false; //郢ｧ・ｿ郢ｧ・､郢晄ｧｭ繝ｻ邵ｺ・ｮ驕伜争辟夂ｹｧ雋槫徐郢ｧ荵昴・郢ｧ蜑・ｽｸﾂ陜玲ｧｭ笆｡邵ｺ莉｣竊鍋ｸｺ蜷ｶ・狗ｸｺ貅假ｽ―ndouble line = 0.0;
int w = 0;
double s = 0.0;//値確認
double ll = 0.0;//wastedistが入るIncreaseAxis
double pp = 0.0;//wastedistが入るAverageChecker
int checker = 0; 
double k = 0.0;//AverageCheckerの値入れ
int po = 0;
// 隴鯉ｽ｢陝・･繝ｻcallback陋幢ｽｴ邵ｺ・ｪ邵ｺ・ｩ邵ｺ・ｧ隴厄ｽｴ隴・ｽｰ邵ｺ霈費ｽ檎ｸｺ・ｦ邵ｺ繝ｻ・玖恆閧ｴ鄂ｲ
}//namespace


RobotDrive::RobotDrive(Encoderma& encoder)
    : Node("robot_drive"), encoders_(encoder), genel_(std::make_shared<Generalva>()) {
    Xof[0] = 0.0;
    Xof[1] = 0.0;
    s = 0.0;
    x = 0.0;
    y = 0.0;
    Cx = 0.0;
    Cy = 0.0;

encoders_.stopAll();
}


RobotDrive::~RobotDrive() {

encoders_.stopAll();
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


    Kakudo2 += GetDegree(turn_count);

    hole = wastedist.load();

if(Kakudo2 == -90) {
Kakudo1 = 270;
}

if(Kakudo2 == -180) {
Kakudo1 = 180;
}      

if(Kakudo2 == -270) {
Kakudo1 = 90;
}
Kakudo1 += Kakudo2;

while (Kakudo1 >= 360) {
      Kakudo1 -= 360;
    }
      
while (Kakudo1 <= -360) {
      Kakudo1 += 360;
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
    
if(encoders_->left) {
po = po * -1;
}
    
encoders_->right = false;
encoders_->left = false;
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
    x = Cx - ((encoders_.GetTotalDistance() - ll) - xyPa);
  }
  else if (deg == 180) {
    xory = true;
    y = Cy - ((encoders_.GetTotalDistance() - ll) - xyPa);
  }
  else if (deg == 270) {
    xory = false;
    x = Cx + ((encoders_.GetTotalDistance() - ll) - xyPa);
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
    

  
    if (pi >= 181 && pi <= 190) {
      static_cast<int>(g);
      checker = g;
      return g;
    }
  }


  return 0;
}//AverageChecker,END

bool RobotDrive::RobotMovement(double xe, double ye) {
 
if(!rclcpp::ok()) return false;
 
if(xe > 600 && xe < 650) {



if(ye >= 0.0 && ye < 50.0) {
RCLCPP_INFO(get_logger(), "moving");


}

}//0 < y < 50
else if(ye > 600.0 && ye < 650.0) {

if(times == 0) {
if(turnrunning == false) {

 encoders_.setspeedstop();

line = encoders_.GetTotalDistance() + 181.0;

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

}//陜玲ｧｭ笆ｲ邵ｺ・ｦ郢ｧ遏ｩﾂ豈費ｽｸ・ｭ邵ｺ・ｫ郢晢ｽ｢郢晢ｽｼ郢ｧ・ｿ郢晢ｽｼ雎・ｽ｢郢ｧ竏壺ｻ邵ｺ・ｻ邵ｺ蜉ｱ・･邵ｺ・ｪ邵ｺ繝ｻ}//times邵ｺ・ｧ闖ｴ蜍溷ｱ馴ｶ・ｮ邵ｺ荵晢ｽ定ｬｨ・ｰ邵ｺ蛹ｻ・矩・・鄂ｰ邵ｺ・ｯ隴厄ｽｲ邵ｺ蠕鯉ｽ狗ｹ晄亢縺・ｹ晢ｽｳ郢晏現窶ｲx900邵ｺ・ｨx1500邵ｺ繧・ｽ狗ｸｺ・ｨ邵ｺ讎翫・邵ｺ・ｫ900邵ｺ・ｮ邵ｺ・ｻ邵ｺ繝ｻ窶ｲ隘搾ｽｷ陷崎ｼ費ｼ邵ｺ・ｦ邵ｺ蜉ｱ竏ｪ邵ｺ繝ｻﾂｰ郢ｧ繝ｻ    }// 1500 < y < 1550
}
}



}//0 < x < 50

else if(xe > 600.0 && xe < 650.0) {

if(ye > 600.0 && ye < 650.0) {
if(times == 1 || times == 8) {
if(turnrunning == false) {

 encoders_.setspeedstop();

line = encoders_.GetTotalDistance() + 181.0;

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
else if(ye > 1500.0 && ye < 1550.0){


if(times == 2) {
if(turnrunning == false) {

 encoders_.setspeedstop();

line = encoders_.GetTotalDistance() + 181.0;

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

else if(xe > 900.0 && xe < 950.0) {

if(ye > 1500.0 && ye < 1550.0) {
if(times == 3) {
if(turnrunning == false) {

 encoders_.setspeedstop();

line = encoders_.GetTotalDistance() + 181.0;

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
else if(ye > 2100.0 && ye < 2150.0){


if(times == 4) {
if(turnrunning == false) {

 encoders_.setspeedstop();

line = encoders_.GetTotalDistance() + 362.0;

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
case 1:
case 2:
case 3:
{

float d = 0; //闕ｳﾂ陜玲ｧｭ笆｡邵ｺ繝ｻ 
    while(true) {
      if(!rclcpp::ok()) return 0.0;

      
    if(d != 0.5f) {
   std::this_thread::sleep_for(std::chrono::milliseconds(1));
    encoders_.RightTurn();
    d = 0.5f;
    
}   

    if(encoders_.GetTotalDistance() >= line) {

     encoders_.setspeedstop();


    result_dist = 181.0;
    break;
    
}

std::this_thread::sleep_for(std::chrono::milliseconds(1));
}//while,end
}
break;
    
case 1:
case 3:
{

float d = 0; //闕ｳﾂ陜玲ｧｭ笆｡邵ｺ繝ｻ 
    while(true) {
      if(!rclcpp::ok()) return 0.0;

      
    if(d != 0.5f) {
   std::this_thread::sleep_for(std::chrono::milliseconds(1));
    encoders_.LeftTurn();
    d = 0.5f;
    
}   

    if(encoders_.GetTotalDistance() >= line) {

     encoders_.setspeedstop();


    result_dist = 181.0;
    break;
    
}

std::this_thread::sleep_for(std::chrono::milliseconds(1));
}//while,end
}
break;    
    
case 4: 
{
 
float d = 0;

    while(true) {     
    if(!rclcpp::ok()) return 0.0;

      if(d != 0.5f) {
 std::this_thread::sleep_for(std::chrono::milliseconds(1));
    encoders_.RightTurn();
    d = 0.5f;
   
}
 
    
    if(encoders_.GetTotalDistance() >= line) {
    
      encoders_.setspeedstop();

    result_dist = 362.0;
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
wastedist.store(wastedist.load() + result_dist); 

if (o == 0) { times = 1; }
else if (o == 1) { times = 2; }
else if (o == 2) { times = 3; }
else if (o == 3) { times = 4; }
else if (o == 4) { times = 5; }
else if (o == 5) { times = 6; } 
else if (o == 6) { times = 7; }
else if (o == 7) { times = 8; } 
else if (o == 8) { times = 9; }
  
  
Mrun = true;
turnrunning = false; 

return result_dist;


}//RobotTurn驍ｨ繧・ｽ冗ｹｧ繝ｻ



void RobotDrive::callback() {

try {
if(!rclcpp::ok()) return;
    


if(set == 0){
std::this_thread::sleep_for(std::chrono::milliseconds(1));
encoders_.setSpeed();

RCLCPP_INFO(get_logger(), "moved");

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
RCLCPP_INFO(get_logger(), "x座標%.2f : y座標%.2f", Xof[0], Xof[1]);
if(!rclcpp::ok()) {

genel_->LWaitingThread();

}

}
catch(const std::exception& errorD) {

RCLCPP_ERROR(this->get_logger(), "エラーが起きました%s", errorD.what());
}
}
