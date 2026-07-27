#include <cmath>
#include "robotdrive.h"
#include <stdexcept>

namespace
{
double count = 0.0;        // 回転したときの走行距離
bool once = false;         // IncreaseAxisの最初だけ行う処理用
int Kakudo = 0;            // 現在の角度
int deg = 0;               // IncreaseAxisで使う角度
double hypotenuse = 0.0;   // 原点までの距離
double DFH = 0.0;          // atan2で出した角度
int AbsDegree = 0;         // 四捨五入後の角度
int turndeg = 0;          // 原点へ向くための回転角度
int result = 0;           // 結果
int add = 0;              // 90度を何回分回ったか
bool turnrunning = false; //機体が回転中か
int times = 0;//何回機体が回転してるか
bool timer = true; //タイマースタートを一回にするため
bool end = false; //タイマーの秒数を取るのを一回だけにするため
double line = 0.0;
int w = 0;

// 既存のcallback側などで更新されている前提
extern bool Mrun;
extern double TotalDistance;
extern double xyPa;

}//namespace


RobotDrive::RobotDrive(Encoderma& encoder) : encoders_(encoder) {

  timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { callback(); });

encoders_.stopAll();
}


RobotDrive::~RobotDrive() {

encoders_.stopAll();    
}


void RobotDrive::TheBeginning() {  
int k = 0;
    
if(d != 0.5) {
   
    encoders_.setSpeed(3, -0.5);
    encoders_.setSpeed(2, 0.5);
    d = 0.5;
    
}   

    if(TotalDistance >= 11.775) {

    encoders_.setSpeed(3, 0.0);
    encoders_.setSpeed(2, 0.0);
    set = 0;
    begin = false;
}
}//TheBeginning,end
    
int RobotDrive::GetDegree_man()
{
  // 機体の回転が終わったら Mrun が true になる
  if (Mrun) {
    // 回転前の距離との差を使う想定
    count = wastedist - hole;
    Kakudo = Kakudo + GetDegree(count);
    hole = wastedist; 
      
    // 0～359度に戻す
    while (Kakudo >= 360) {
      Kakudo -= 360;
    }

    Mrun = false;
  }

  return Kakudo;
} //GetDegree_man,END


int RobotDrive::GetDegree(double turnDistance)
{
  // 例: 23 ÷ 2 = 11.5 なので add = 2
  add = AverageChecker(turnDistance);

  // 次の直進区間では、現在位置を基準にする
  once = true;

  // 2 × 90 = 180度
  return add * 90;
}//GetDegree,END


std::array<double, 2> RobotDrive::IncreaseAxis(int degrees)
{
     
// 旋回直後、最初の一回だけ基準位置と基準距離を保存
  if (once) {
    deg = degrees;
    xyPa = TotalDistance - wastedist;

    Cx = x;
    Cy = y;

    once = false;
  }

  // 右回転だけを使う場合でも、
  // 現在角度により x/y と +/− を決める。
  if (deg == 0) {
    xory = true;
    y = Cy + (TotalDistance - xyPa);
  }
  else if (deg == 90) {
    xory = false;
    x = Cx + (TotalDistance - xyPa);
  }
  else if (deg == 180) {
    xory = true;
    y = Cy - (TotalDistance - xyPa);
  }
  else if (deg == 270) {
    xory = false;
    x = Cx - (TotalDistance - xyPa);
  }

  return {x, y};



}//IncreaseAxis,END

int RobotDrive::returnToOrigin(double Xe, double Ye)
{
  // 原点までの直線距離
  hypotenuse = std::sqrt((Xe * Xe) + (Ye * Ye));

  // atan2の結果はラジアンなので、度へ変換する
  DFH = std::atan2(Ye, Xe) * 180.0 / M_PI;

  // 整数の角度に丸める
  AbsDegree = static_cast<int>(std::round(DFH));

  // ここは後で原点復帰の回転ルールを詰める場所
  turndeg = 180 - (90 + AbsDegree);

  if (Xe < 0) {
    result = 360 - turndeg;
  }
  else {
    result = turndeg;
  }

  // 0～359度に直す
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
    // intではなくdouble。11.5なども判定できるようにする。
    const double k = dd / static_cast<double>(g);

    // 1回の90度旋回に必要な距離が11～12なら採用
    if (k >= 11.0 && k <= 12.0) {
      return g;
    }
  }

  // 想定した範囲に入らなかった場合
  return 0;
}//AverageChecker,END

bool RobotDrive::RobotMovement() {
    
 
else if(x > -600 && x < -650) {

if(y > 0 && y < 50){


if(times == 0) {
    
if(turnrunning == false) {
encoders_.setSpeed(3, 0.0);
encoders_.setSpeed(2, 0.0);
line = TotalDistance + 35.325;

turnrunning = true;
int current_w = times;
times = -1;

  
if(turn_thread.joinable()) {
        turn_thread.join(); 
    }

Turna_thread = std::thread([w = current_w]() {
RobotTurn(w); 
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
line = TotalDistance + 23.55;

turnrunning = true;
int current_w = times;
times = -1;

  
if(turn_thread.joinable()) {
        turn_thread.join(); 
    }

Turna_thread = std::thread([w = 1]() {
RobotTurn(w); 
    });

std::this_thread::sleep_for(std::chrono::milliseconds(40));

}//回ってる途中にモーター止めてほしくない
}//timesで何回目かを数える理由は曲がるポイントがx900とx1500あるとき先に900のほうが起動してしまうから
    }// 1500 < y < 1550
else if(y > 900 && y < 950){



if(times == 2) {
    
if(turnrunning == false) {
encoders_.setSpeed(3, 0.0);
encoders_.setSpeed(2, 0.0);
line = TotalDistance + 11.775;

turnrunning = true;
int current_w = times;
times = -1;

  
if(turn_thread.joinable()) {
        turn_thread.join(); 
    }

Turna_thread = std::thread([w = current_w]() {
RobotTurn(w); 
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
line = TotalDistance + 11.775;

turnrunning = true;
     int current_w = times; 
      times = -1; 

      if(turn_thread.joinable()) {
          turn_thread.join();
      }

      turn_thread = std::thread([w = current_w]() { 
          RobotTurn(w); 
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
line = TotalDistance + 23.55;

turnrunning = true;
int current_w = times;
times = -1;

  
if(turn_thread.joinable()) {
        turn_thread.join(); 
    }

Turna_thread = std::thread([w = current_w]() {
RobotTurn(w); 
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
line = TotalDistance + 11.775;

turnrunning = true;
int current_w = times;
times = -6;

  
if(turn_thread.joinable()) {
        turn_thread.join(); 
    }

Turna_thread = std::thread([w = current_w]() {
RobotTurn(w); 
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
line = TotalDistance + 23.55;

turnrunning = true;
int current_w = times;
times = -1;

  
if(turn_thread.joinable()) {
        turn_thread.join(); 
    }

Turna_thread = std::thread([w = 7]() {
RobotTurn(w); 
    });

std::this_thread::sleep_for(std::chrono::milliseconds(40));

}


}


    
} // 1500 < y <1550




}// 600 < x < 650

return true;//murnに入れる
}

double RobotDrive::RobotTurn(int o) {
  float d = 0;
  double result_dist = 0.0;
    
//色によって変わるが例えば赤
switch(o) {
case 2:
case 3:
case 5:
case 6:

float d = 0; //一回だけ
 
    while(true) {     
    if(d != 0.5f) {
   
    encoders_.setSpeed(3, -0.5);
    encoders_.setSpeed(2, 0.5);
    d = 0.5f;
    
}   

    if(TotalDistance >= line) {

    encoders_.setSpeed(3, 0.0);
    encoders_.setSpeed(2, 0.0);


    result_dist = 11.775;
    
}

std::this_thread::sleep_for(std::chrono::milliseconds(1));
}//while,end
break;

    
case 1:
case 4:
case 7: 
 
float d = 0;

    while(true) {     
    if(d != 0.5f) {
 
    encoders_.setSpeed(3, -0.5);
    encoders_.setSpeed(2, 0.5);
    d = 0.5f;
   
}
 
    
    if(TotalDistance >= line) {
    encoders_.setSpeed(3, 0.0);
    encoders_.setSpeed(2, 0.0);


    result_dist = 23.55;
}

std::this_thread::sleep_for(std::chrono::milliseconds(1));
}//while,end
break;

case 0: 
float d = 0;
    while(true) {     
    if(d != 0.5f) {

    encoders_.setSpeed(3, -0.5);
    encoders_.setSpeed(2, 0.5);
    d = 0.5f;
}

        
    if(TotalDistance >= line) {
    encoders_.setSpeed(3, 0.0);
    encoders_.setSpeed(2, 0.0);
    result_dist = 35.325;  
    
}
std::this_thread::sleep_for(std::chrono::milliseconds(1));
}//while,end

break;

default :
  break;

}//switch終わり

set = 0;
wastedist += result_dist; 

if (w == 0)      { times = 1; }
else if (w == 1) { times = 2; }
else if (w == 2) { times = 3; }
else if (w == 3) { times = 4; }
else if (w == 4) { times = 5; }
else if (w == 5) { times = 6; } 
else if (w == 6) { times = 7; }
else if (w == 7) { times = 8; } 

  
  
turnrunning = false; 

return result_dist;


}//RobotTurn終わり

void RobotDrive::callback() {

try {
  
if(begin) {

The Beginning();

}
    

if(set == 0){
encoders_.setSpeed(3, 0.5)
encoders_.setSpeed(2, 0.5)
set = 1;
}

if(TheBeginning == false) {
    
if(turnrunning == false) {     
Marn = RobotMovement();

delivery = GetDegree_man();

Xof = IncreaseAxis(delivery);
}
}
RCLCPP_INFO(get_logger(), "x座標 %.2f : y座標 %.2f", Xof[0], Xof[1]);

}
catch(const std::exception& errorD) {

RCLCPP_ERROR(this->get_logger(), "エラーが発生しました %s", errorD.what());
}

}


