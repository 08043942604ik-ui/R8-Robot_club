#include <cmath>
#include "robotdrive.h"
#include <stdexcept>

namespace
{
constexpr double kPi = 3.14159265358979323846;
int Kakudo = 90;
double turn_count = 0.0;   // 回転したときの走行距離
bool once = false;         // IncreaseAxis邵ｺ・ｮ隴崢陋ｻ譏ｴ笆｡邵ｺ鬘鯉ｽ｡蠕娯鴬陷・ｽｦ騾・・逡曾nint Kakudo = 0;            // 霑ｴ・ｾ陜ｨ・ｨ邵ｺ・ｮ髫苓ｲ橸ｽｺ・ｦ
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

// 隴鯉ｽ｢陝・･繝ｻcallback陋幢ｽｴ邵ｺ・ｪ邵ｺ・ｩ邵ｺ・ｧ隴厄ｽｴ隴・ｽｰ邵ｺ霈費ｽ檎ｸｺ・ｦ邵ｺ繝ｻ・玖恆閧ｴ鄂ｲ
}//namespace


RobotDrive::RobotDrive(Encoderma& encoder)
    : Node("robot_drive"), encoders_(encoder) {


encoders_.stopAll();
}


RobotDrive::~RobotDrive() {

encoders_.stopAll();
if (Turna_thread.joinable()) {
  Turna_thread.join();
}
}


void RobotDrive::TheBeginning() {  
int k = 0;
    
if(d != 0.5) {
   
    encoders_.setSpeed(3, -0.1);
    encoders_.setSpeed(2, -0.1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    d = 0.5;
    
}   

    if(encoders_.GetTotalDistance() >= 117.7) {

    encoders_.setSpeed(3, 0.0);
    encoders_.setSpeed(2, 0.0);
    set = 0;
    begin = false;
}
}//TheBeginning,end
    
int RobotDrive::GetDegree_man()
{
  if (Mrun) {
    turn_count = wastedist.load() - hole;
  // 隶匁ｻ会ｽｽ阮吶・陜玲ｫ・ｽｻ・｢邵ｺ讙趣ｽｵ繧・ｽ冗ｸｺ・｣邵ｺ貅假ｽ・Mrun 邵ｺ繝ｻtrue 邵ｺ・ｫ邵ｺ・ｪ郢ｧ繝ｻ  if (Mrun) {
    // 陜玲ｫ・ｽｻ・｢陷鷹亂繝ｻ髴肴辨螻ｬ邵ｺ・ｨ邵ｺ・ｮ陝ｾ・ｮ郢ｧ蜑・ｽｽ・ｿ邵ｺ繝ｻﾎｦ陞ｳ繝ｻ    count = wastedist - hole;
    Kakudo = Kakudo + GetDegree(turn_count);
    hole = wastedist.load();
      
    // 0繝ｻ繝ｻ59陟趣ｽｦ邵ｺ・ｫ隰鯉ｽｻ邵ｺ繝ｻ    while (Kakudo >= 360) {
    while (Kakudo >= 360) {
      Kakudo -= 360;
    }

    Mrun = false;
  }

  return Kakudo;
} //GetDegree_man,END


int RobotDrive::GetDegree(double turnDistance)
{
  // 關薙・ 23 ・・ｽｷ 2 = 11.5 邵ｺ・ｪ邵ｺ・ｮ邵ｺ・ｧ add = 2
  add = AverageChecker(turnDistance);
  once = true;

  // 隹ｺ・｡邵ｺ・ｮ騾ｶ・ｴ鬨ｾ・ｲ陋ｹ・ｺ鬮｢阮吶堤ｸｺ・ｯ邵ｲ竏ｫ讓溯舉・ｨ闖ｴ蜥ｲ・ｽ・ｮ郢ｧ雋樊ｸ戊ｲ・じ竊鍋ｸｺ蜷ｶ・欺n  once = true;

  // 2 ・・・90 = 180陟趣ｽｦ
  return add * 90;
}//GetDegree,END


std::array<double, 2> RobotDrive::IncreaseAxis(int degrees)
{
  if (once) {
     
// 隴悟唱螻馴ｶ・ｴ陟募ｾ個竏ｵ諤呵崕譏ｴ繝ｻ闕ｳﾂ陜玲ｧｭ笆｡邵ｺ螟ｧ貂戊ｲ・ｴ具ｽｽ蜥ｲ・ｽ・ｮ邵ｺ・ｨ陜難ｽｺ雋・・・ｷ譎槫ｱｬ郢ｧ蜑・ｽｿ譎擾ｽｭ繝ｻ  
    deg = degrees;
    xyPa = encoders_.GetTotalDistance() - wastedist.load();

    Cx = x;
    Cy = y;

    once = false;
  }

  // 陷ｿ・ｳ陜玲ｫ・ｽｻ・｢邵ｺ・ｰ邵ｺ莉｣・定抄・ｿ邵ｺ繝ｻ・ｰ・ｴ陷ｷ蛹ｻ縲堤ｹｧ繧・繝ｻ  // 霑ｴ・ｾ陜ｨ・ｨ髫苓ｲ橸ｽｺ・ｦ邵ｺ・ｫ郢ｧ蛹ｻ・・x/y 邵ｺ・ｨ +/遶上・郢ｧ蜻茨ｽｱ・ｺ郢ｧ竏夲ｽ狗ｸｲ繝ｻ  if (deg == 0) {
  if (deg == 0) {
    xory = true;
    y = Cy + (encoders_.GetTotalDistance() - xyPa);
  }
  else if (deg == 90) {
    xory = false;
    x = Cx + (encoders_.GetTotalDistance() - xyPa);
  }
  else if (deg == 180) {
    xory = true;
    y = Cy - (encoders_.GetTotalDistance() - xyPa);
  }
  else if (deg == 270) {
    xory = false;
    x = Cx - (encoders_.GetTotalDistance() - xyPa);
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

  if (Xe < 0) {
    result = 360 - turndeg;
  }
  else {
    result = turndeg;
  }

  // 0繝ｻ繝ｻ59陟趣ｽｦ邵ｺ・ｫ騾ｶ・ｴ邵ｺ繝ｻ  while (result < 0) {
  while (result < 0) {
    result += 360;
  }

  while (result >= 360) {
    result -= 360;
  }

  return result;
}//returnToOrigin,END


int RobotDrive::AverageChecker(double dd)
{
  for (int g = 1; g < 4; g++) {
    const double k = dd / static_cast<double>(g);
    // int邵ｺ・ｧ邵ｺ・ｯ邵ｺ・ｪ邵ｺ驥腔uble邵ｲ繝ｻ1.5邵ｺ・ｪ邵ｺ・ｩ郢ｧ繧・・陞ｳ螢ｹ縲堤ｸｺ髦ｪ・狗ｹｧ蛹ｻ竕ｧ邵ｺ・ｫ邵ｺ蜷ｶ・狗ｸｲ繝ｻ    const double k = dd / static_cast<double>(g);

    // 1陜玲ｧｭ繝ｻ90陟趣ｽｦ隴悟唱螻鍋ｸｺ・ｫ陟｢繝ｻ・ｦ竏壺・髴肴辨螻ｬ邵ｺ繝ｻ1繝ｻ繝ｻ2邵ｺ・ｪ郢ｧ逕ｻ豐ｻ騾包ｽｨ
    if (k >= 110.0 && k <= 120.0) {
      return g;
    }
  }

  // 隲・ｳ陞ｳ螢ｹ・邵ｺ貅ｽ・ｯ繝ｻ蟲・ｸｺ・ｫ陷茨ｽ･郢ｧ蟲ｨ竊醍ｸｺ荵昶夢邵ｺ貅ｷ・ｰ・ｴ陷ｷ繝ｻ  return 0;
  return 0;
}//AverageChecker,END

bool RobotDrive::RobotMovement() {
    
 
if(x > -650 && x < -600) {

if(y > 0 && y < 50){


if(times == 0) {
    
if(turnrunning == false) {
encoders_.setSpeed(3, 0.0);
encoders_.setSpeed(2, 0.0);
line = encoders_.GetTotalDistance() + 353.25;

turnrunning = true;
int current_w = times;
times = -1;

  
if(Turna_thread.joinable()) {
        Turna_thread.join(); 
    }

Turna_thread = std::thread([this, current_w]() {
RobotTurn(current_w);
    });   

    
std::this_thread::sleep_for(std::chrono::milliseconds(40));

}
    
}


}//0 < y < 50


else if(y > 1500 && y < 1550) {

if(times == 1) {
if(turnrunning == false) {
encoders_.setSpeed(3, 0.0);
encoders_.setSpeed(2, 0.0);
line = encoders_.GetTotalDistance() + 235.5;

turnrunning = true;
int current_w = times;
times = -1;

  
if(Turna_thread.joinable()) {
        Turna_thread.join(); 
    }

Turna_thread = std::thread([this, current_w]() {
RobotTurn(current_w);
    });

std::this_thread::sleep_for(std::chrono::milliseconds(40));

}//陜玲ｧｭ笆ｲ邵ｺ・ｦ郢ｧ遏ｩﾂ豈費ｽｸ・ｭ邵ｺ・ｫ郢晢ｽ｢郢晢ｽｼ郢ｧ・ｿ郢晢ｽｼ雎・ｽ｢郢ｧ竏壺ｻ邵ｺ・ｻ邵ｺ蜉ｱ・･邵ｺ・ｪ邵ｺ繝ｻ}//times邵ｺ・ｧ闖ｴ蜍溷ｱ馴ｶ・ｮ邵ｺ荵晢ｽ定ｬｨ・ｰ邵ｺ蛹ｻ・矩・・鄂ｰ邵ｺ・ｯ隴厄ｽｲ邵ｺ蠕鯉ｽ狗ｹ晄亢縺・ｹ晢ｽｳ郢晏現窶ｲx900邵ｺ・ｨx1500邵ｺ繧・ｽ狗ｸｺ・ｨ邵ｺ讎翫・邵ｺ・ｫ900邵ｺ・ｮ邵ｺ・ｻ邵ｺ繝ｻ窶ｲ隘搾ｽｷ陷崎ｼ費ｼ邵ｺ・ｦ邵ｺ蜉ｱ竏ｪ邵ｺ繝ｻﾂｰ郢ｧ繝ｻ    }// 1500 < y < 1550
}
}
else if(y > 900 && y < 950){



if(times == 2) {
    
if(turnrunning == false) {
encoders_.setSpeed(3, 0.0);
encoders_.setSpeed(2, 0.0);
line = encoders_.GetTotalDistance() + 117.75;

turnrunning = true;
int current_w = times;
times = -1;

  
if(Turna_thread.joinable()) {
        Turna_thread.join(); 
    }

Turna_thread = std::thread([this, current_w]() {
RobotTurn(current_w);
    });

    
std::this_thread::sleep_for(std::chrono::milliseconds(40));

}
    
}



}


}//-600 < x < -650

else if(x > 0 && x < 50) {

if(y > 900 && y < 950) {
if(times == 3 || times == 5) {
if(turnrunning == false) {

encoders_.setSpeed(3, 0.0);
encoders_.setSpeed(2, 0.0);
line = encoders_.GetTotalDistance() + 117.75;

turnrunning = true;
     int current_w = times; 
      times = -1; 

if(Turna_thread.joinable()) {
          Turna_thread.join();
      }

      Turna_thread = std::thread([this, current_w]() {
          RobotTurn(current_w);
      });

  


std::this_thread::sleep_for(std::chrono::milliseconds(40));
    

}

}

} //900 < y < 950
else if(y > 1500 && y < 1550){


if(times == 4) {
if(turnrunning == false) {

encoders_.setSpeed(3, 0.0);
encoders_.setSpeed(2, 0.0);
line = encoders_.GetTotalDistance() + 235.5;

turnrunning = true;
int current_w = times;
times = -1;

  
if(Turna_thread.joinable()) {
        Turna_thread.join(); 
    }

Turna_thread = std::thread([this, current_w]() {
RobotTurn(current_w);
    });

std::this_thread::sleep_for(std::chrono::milliseconds(40));
    
}


}


    
} // 1500 < y <1550
} // 0 < x < 50

else if(x > 600 && x < 650) {

if(y > 900 && y < 950) {
if(times == 6) {
if(turnrunning == false) {

encoders_.setSpeed(3, 0.0);
encoders_.setSpeed(2, 0.0);
line = encoders_.GetTotalDistance() + 117.75;

turnrunning = true;
int current_w = times;
times = -6;

  
if(Turna_thread.joinable()) {
        Turna_thread.join(); 
    }

Turna_thread = std::thread([this, current_w]() {
RobotTurn(current_w);
    });

std::this_thread::sleep_for(std::chrono::milliseconds(40));

}

}

} // 900 < y < 950
else if(y > 1500 && y < 1550){


if(times == 7) {
if(turnrunning == false) {

encoders_.setSpeed(3, 0.0);
encoders_.setSpeed(2, 0.0);
line = encoders_.GetTotalDistance() + 235.5;

turnrunning = true;
int current_w = times;
times = -1;

  
if(Turna_thread.joinable()) {
        Turna_thread.join(); 
    }

Turna_thread = std::thread([this, current_w]() {
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
  float d = 0;
  double result_dist = 0.0;
    
//豼ｶ・ｲ邵ｺ・ｫ郢ｧ蛹ｻ笆ｲ邵ｺ・ｦ陞溷ｳｨ・冗ｹｧ荵昶ｲ關謎ｹ昶斡邵ｺ・ｰ隘搾ｽ､
switch(o) {
case 2:
case 3:
case 5:
case 6:
{

float d = 0; //闕ｳﾂ陜玲ｧｭ笆｡邵ｺ繝ｻ 
    while(true) {     
    if(d != 0.5f) {
   
    encoders_.setSpeed(3, -0.1);
    encoders_.setSpeed(2, 0.1);
    d = 0.5f;
    
}   

    if(encoders_.GetTotalDistance() >= line) {

    encoders_.setSpeed(3, 0.0);
    encoders_.setSpeed(2, 0.0);


    result_dist = 117.75;
    break;
    
}

std::this_thread::sleep_for(std::chrono::milliseconds(1));
}//while,end
}
break;

    
case 1:
case 4:
case 7: 
{
 
float d = 0;

    while(true) {     
    if(d != 0.5f) {
 
    encoders_.setSpeed(3, -0.1);
    encoders_.setSpeed(2, 0.1);
    d = 0.5f;
   
}
 
    
    if(encoders_.GetTotalDistance() >= line) {
    encoders_.setSpeed(3, 0.0);
    encoders_.setSpeed(2, 0.0);


    result_dist = 235.5;
    break;
}

std::this_thread::sleep_for(std::chrono::milliseconds(1));
}//while,end
}
break;

case 0: 
{
float d = 0;
    while(true) {     
    if(d != 0.5f) {

    encoders_.setSpeed(3, -0.1);
    encoders_.setSpeed(2, 0.1);
    d = 0.5f;
}

        
    if(encoders_.GetTotalDistance() >= line) {
    encoders_.setSpeed(3, 0.0);
    encoders_.setSpeed(2, 0.0);
    result_dist = 353.25;  
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

if (o == 0)      { times = 1; }
else if (o == 1) { times = 2; }
else if (o == 2) { times = 3; }
else if (o == 3) { times = 4; }
else if (o == 4) { times = 5; }
else if (o == 5) { times = 6; } 
else if (o == 6) { times = 7; }
else if (o == 7) { times = 8; } 

  
  
Mrun = true;
turnrunning = false; 

return result_dist;


}//RobotTurn驍ｨ繧・ｽ冗ｹｧ繝ｻ
void RobotDrive::callback() {

try {
  
if(begin) {

TheBeginning();

}
    


if(set == 0){
encoders_.setSpeed(3, 0.1);
encoders_.setSpeed(2, -0.1);

std::this_thread::sleep_for(std::chrono::milliseconds(10));

set = 1;
}

if(begin == false) {
    
if(turnrunning == false) {     
Marn = RobotMovement();

delivery = GetDegree_man();

Xof = IncreaseAxis(delivery);
}
}
RCLCPP_INFO(get_logger(), "x陟趣ｽｧ隶薙・%.2f : y陟趣ｽｧ隶薙・%.2f", Xof[0], Xof[1]);

}
catch(const std::exception& errorD) {

RCLCPP_ERROR(this->get_logger(), "郢ｧ・ｨ郢晢ｽｩ郢晢ｽｼ邵ｺ讙主験騾墓ｺ假ｼ邵ｺ・ｾ邵ｺ蜉ｱ笳・%s", errorD.what());
}

}
