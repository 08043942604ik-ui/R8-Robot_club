//こっからは時間で回転するとき

#include "titan.h"
#include <chrono>
#include <thread>
#include "Encoder.h"
#include <array>
#include <cmath>

namespace
{

bool xory;                 // x または y の判断用
double x = 0.0;            // x座標
double y = 0.0;            // y座標
double count = 0.0;        // 回転したときの走行距離
bool once = false;         // IncreaseAxisの最初だけ行う処理用
int Kakudo = 0;            // 現在の角度
int deg = 0;               // IncreaseAxisで使う角度
double hypotenuse = 0.0;   // 原点までの距離
double DFH = 0.0;          // atan2で出した角度
int AbsDegree = 0;         // 四捨五入後の角度
int turndeg = 0;          // 原点へ向くための回転角度
int result = 0;           // 結果
double Cx = 0.0;          // 区間開始時のx座標
double Cy = 0.0;          // 区間開始時のy座標
int add = 0;              // 90度を何回分回ったか
int times = 0; //何回目か
bool turnrunning = false; //機体が回転中か
int times = 0;//何回機体が回転してるか
bool timer = true; //タイマースタートを一回にするため
bool end = false; //タイマーの秒数を取るのを一回だけにするため

// 既存のcallback側などで更新されている前提
extern bool Mrun;
extern double TotalDistance;
extern double xyPa;

int GetDegree_man();
int GetDegree(double turnDistance);
std::array<double, 2> IncreaseAxis(int degrees);
int returnToOrigin(double Xe, double Ye);
int AverageChecker(double dd);
int RobotMovement();
double turnDistanceChecker();//totalの距離と引く数を取るため
    
} // namespace


int GetDegree_man()
{
  // 機体の回転が終わったら Mrun が true になる
  if (Mrun) {
    // 回転前の距離との差を使う想定
    count = TotalDistance;//turnDistanceCheckerで帰ってきた値引く
    Kakudo = Kakudo + GetDegree(count);

    // 0～359度に戻す
    while (Kakudo >= 360) {
      Kakudo -= 360;
    }

    Mrun = false;
  }

  return Kakudo;
}


int GetDegree(double turnDistance)
{
  // 例: 23 ÷ 2 = 11.5 なので add = 2
  add = AverageChecker(turnDistance);

  // 次の直進区間では、現在位置を基準にする
  once = true;

  // 2 × 90 = 180度
  return add * 90;
}


std::array<double, 2> IncreaseAxis(int degrees)
{
  // 旋回直後、最初の一回だけ基準位置と基準距離を保存
  if (once) {
    deg = degrees;
    xyPa = TotalDistance - /*回転のせいで増えた数引く*/;

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
}


int returnToOrigin(double Xe, double Ye)
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
}


int AverageChecker(double dd)
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
}

int RobotMovement() {

RealDistance = Totaldistance - TotalTdistance;
 
if(x > -600 && x < -650) {

if(y > 1500 && y < 1550) {

if(times == 0) {
if(turnrunning == false) {

leftmotor.SetSpeed(0.0);
rightmotor.SetSpeed(0.0);

RobotTurn(times);

std::this_thread::sleep_for(std::chrono::milliseconds(40));
++times;
}//回ってる途中にモーター止めてほしくない
}//timesで何回目かを数える理由は曲がるポイントがx900とx1500あるとき先に900のほうが起動してしまうから
    }// 1500 < y < 1550
else if(y > 900 && y < 950){



if(times == 1) {
    
if(turnrunning == false) {

leftmotor.SetSpeed(0.0);
rightmotor.SetSpeed(0.0);
RobotTurn(times);

std::this_thread::sleep_for(std::chrono::milliseconds(40));
++times;
}
    
}



}


}//-600 < x < -650

else if(x > 0 && x < 50) {

if(y > 900 && y < 950) {
if(times == 2 && times == 4) {
if(turnrunning == false) {

leftmotor.SetSpeed(0.0);
rightmotor.SetSpeed(0.0);
RobotTurn(times);
std::this_thread::sleep_for(std::chrono::milliseconds(40));
    
++times;
}

}

} //900 < y < 950
else if(y > 1500 && y < 1550){


if(times == 3) {
if(turnrunning == false) {

leftmotor.SetSpeed(0.0);
rightmotor.SetSpeed(0.0);
RobotTurn(times);

std::this_thread::sleep_for(std::chrono::milliseconds(40));

++times;
}


}


    
} // 1500 < y <1550
} // 0 < x < 50

else if(x > 600 && < x < 650) {

if(y > 900 && y < 950) {
if(times == 5) {
if(turnrunning == false) {

leftmotor.SetSpeed(0.0);
rightmotor.SetSpeed(0.0);
RobotTurn(times);

std::this_thread::sleep_for(std::chrono::milliseconds(40));
++times;
}

}

} // 900 < y < 950
else if(y > 1500 && y < 1550){


if(times == 6) {
if(turnrunning == false) {

leftmotor.SetSpeed(0.0);
rightmotor.SetSpeed(0.0);
RobotTurn(times);

std::this_thread::sleep_for(std::chrono::milliseconds(40));
++times;
}


}


    
} // 1500 < y <1550




}// 600 < x < 650

bool RobotTurn(int d) {
//色によって変わるが例えば赤
switch(d) {

case 1:
case 2:
case 4:
case 5:

   turnrunning = true;
float d = 0; //一回だけ
 
    while(true) {     
    if(d != 0.5) {
    auto start1 = std::chrono::high_resolution_clock::now(); 
    SetSpeed(leftmotor, -0.5);
    SetSpeed(rightmotor, 0.5);
    d = 0.5;
      
}   
 auto end1 = std::chrono::high_resolution_clock::now();
 auto end_time1 == std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);

    if(end_time1 == /*90°の秒数*/) {

    SetSpeed(leftmotor, 0.0);
    SetSpeed(rightmotor, 0.0);
    end = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); //endをtrueにしてから処理する時間を
    
    turnrunning = false;
    break;
}

        
}//while,end
case 0:
case 3:
case 6: 
 
turnrunning = true;
float d = 0;

    while(true) {     
    if(d != 0.5) {
 auto start2 = std::chrono::high_resolution_clock::now();     
    SetSpeed(leftmotor, -0.5);
    SetSpeed(rightmotor, 0.5);
    d = 0.5;
   
}
    auto end2 = std::chrono::high_resolution_clock::now();
 auto end_time2 == std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2);
    
    if(end_time2 == /*180°の秒数*/)) {
    SetSpeed(leftmotor, 0.0);
    SetSpeed(rightmotor, 0.0);
    end = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    
    turnrunning = false;
    break;
}

}//while,end
default : 
   turnrunning = true;
float d = 0;
    while(true) {     
    if(d != 0.5) {
    auto start3 = std::chrono::high_resolution_clock::now();
    SetSpeed(leftmotor, -0.5);
    SetSpeed(rightmotor, 0.5);
    d = 0.5;
}
auto end3 = std::chrono::high_resolution_clock::now();
auto end_time3 == std::chrono::duration_cast<std::chrono::milliseconds>(end3 - start3);
        
    if(end_time3 == /*270°の秒数*/) {
    SetSpeed(leftmotor, 0.0);
    SetSpeed(rightmotor, 0.0);
    end = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    
    turnrunning = false;
    break;
}
}//while,end


}//switch終わり
return true;

}//RobotTurn終わり

//turnrunningがtrueになったら
double turnDistancechecker() //回転でどんだけ動いたかを送る
if(timer) {
auto start_time = std::chrono::high_resolution_clock::now();

timer = false;
} //timerがtrueになるのを回転の関数のifの中に入れとくのとcallbackの中でこの関数をif(turnrunning)で囲んどく

if(end) {
auto end_time = std::chrono::high_resolution_clock::now();

auto seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

end = false;
}

if(/*ここに90°回ったときの秒数の範囲*/){}//これらの中で返す値を変える

if(/*ここは180°*/){}

if(/*ここは270°*/){}


}

//こっからは距離
#include "titan.h"
#include <chrono>
#include <thread>
#include "Encoder.h"
#include <array>
#include <cmath>

namespace
{

bool xory;                 // x または y の判断用
double x = 0.0;            // x座標
double y = 0.0;            // y座標
double count = 0.0;        // 回転したときの走行距離
bool once = false;         // IncreaseAxisの最初だけ行う処理用
int Kakudo = 0;            // 現在の角度
int deg = 0;               // IncreaseAxisで使う角度
double hypotenuse = 0.0;   // 原点までの距離
double DFH = 0.0;          // atan2で出した角度
int AbsDegree = 0;         // 四捨五入後の角度
int turndeg = 0;          // 原点へ向くための回転角度
int result = 0;           // 結果
double Cx = 0.0;          // 区間開始時のx座標
double Cy = 0.0;          // 区間開始時のy座標
int add = 0;              // 90度を何回分回ったか
int times = 0; //何回目か
bool turnrunning = false; //機体が回転中か
int times = 0;//何回機体が回転してるか
bool timer = true; //タイマースタートを一回にするため
bool end = false; //タイマーの秒数を取るのを一回だけにするため
double wastedist = 0.0;//無駄で引かなきゃいけない数
double hole = 0.0;

// 既存のcallback側などで更新されている前提
extern bool Mrun;
extern double TotalDistance;
extern double xyPa;

int GetDegree_man();
int GetDegree(double turnDistance);
std::array<double, 2> IncreaseAxis(int degrees);
int returnToOrigin(double Xe, double Ye);
int AverageChecker(double dd);
int RobotMovement();
bool RobotTurn();
    
} // namespace


int GetDegree_man()
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
}


int GetDegree(double turnDistance)
{
  // 例: 23 ÷ 2 = 11.5 なので add = 2
  add = AverageChecker(turnDistance);

  // 次の直進区間では、現在位置を基準にする
  once = true;

  // 2 × 90 = 180度
  return add * 90;
}


std::array<double, 2> IncreaseAxis(int degrees)
{
if(turnrunning == false) { 
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
}
}

int returnToOrigin(double Xe, double Ye)
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
}


int AverageChecker(double dd)
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
}

int RobotMovement() {

 
if(x > -600 && x < -650) {

if(y > 0 && y < 50){


if(times == 0) {
    
if(turnrunning == false) {
leftmotor.SetSpeed(0.0);
rightmotor.SetSpeed(0.0);
line = Totaldistance + 35.325;
wastedist += RobotTurn(times);

    
std::this_thread::sleep_for(std::chrono::milliseconds(40));
++times;
}
    
}


}//0 < y < 50


else if(y > 1500 && y < 1550) {

if(times == 1) {
if(turnrunning == false) {
leftmotor.SetSpeed(0.0);
rightmotor.SetSpeed(0.0);
line = Totaldistance + 23.55
wastedist += RobotTurn(times);

std::this_thread::sleep_for(std::chrono::milliseconds(40));
++times;
}//回ってる途中にモーター止めてほしくない
}//timesで何回目かを数える理由は曲がるポイントがx900とx1500あるとき先に900のほうが起動してしまうから
    }// 1500 < y < 1550
else if(y > 900 && y < 950){



if(times == 2) {
    
if(turnrunning == false) {
leftmotor.SetSpeed(0.0);
rightmotor.SetSpeed(0.0);
line = Totaldistance + 11.775;
wastedist += RobotTurn(times);

    
std::this_thread::sleep_for(std::chrono::milliseconds(40));
++times;
}
    
}



}


}//-600 < x < -650

else if(x > 0 && x < 50) {

if(y > 900 && y < 950) {
if(times == 3 && times == 5) {
if(turnrunning == false) {

leftmotor.SetSpeed(0.0);
rightmotor.SetSpeed(0.0);
line = Totaldistance + 11.775;
wastedist += RobotTurn(times);
std::this_thread::sleep_for(std::chrono::milliseconds(40));
    
++times;
}

}

} //900 < y < 950
else if(y > 1500 && y < 1550){


if(times == 4) {
if(turnrunning == false) {

leftmotor.SetSpeed(0.0);
rightmotor.SetSpeed(0.0);
line = Totaldistance + 23.55
wastedist += RobotTurn(times);

std::this_thread::sleep_for(std::chrono::milliseconds(40));

++times;
}


}


    
} // 1500 < y <1550
} // 0 < x < 50

else if(x > 600 && < x < 650) {

if(y > 900 && y < 950) {
if(times == 6) {
if(turnrunning == false) {

leftmotor.SetSpeed(0.0);
rightmotor.SetSpeed(0.0);
line = Totaldistance + 11.775;
wastedist += RobotTurn(times);

std::this_thread::sleep_for(std::chrono::milliseconds(40));
++times;
}

}

} // 900 < y < 950
else if(y > 1500 && y < 1550){


if(times == 7) {
if(turnrunning == false) {

leftmotor.SetSpeed(0.0);
rightmotor.SetSpeed(0.0);
line = Totaldistance + 23.55
wastedist += RobotTurn(times);

std::this_thread::sleep_for(std::chrono::milliseconds(40));
++times;
}


}


    
} // 1500 < y <1550




}// 600 < x < 650

return true;//murnに入れる
}

double RobotTurn(int w) {
//色によって変わるが例えば赤
switch(d) {
case 2:
case 3:
case 5:
case 6:

   turnrunning = true;
float d = 0; //一回だけ
 
    while(true) {     
    if(d != 0.5) {
   
    SetSpeed(leftmotor, -0.5);
    SetSpeed(rightmotor, 0.5);
    d = 0.5;
      
}   

    if(totaldistance == line) {

    SetSpeed(leftmotor, 0.0);
    SetSpeed(rightmotor, 0.0);


    
    turnrunning = false;
    return 11.775; 
}

        
}//while,end


    
case 1:
case 4:
case 7: 
 
turnrunning = true;
float d = 0;

    while(true) {     
    if(d != 0.5) {
 
    SetSpeed(leftmotor, -0.5);
    SetSpeed(rightmotor, 0.5);
    d = 0.5;
   
}
 
    
    if(totaldistance == line)) {
    SetSpeed(leftmotor, 0.0);
    SetSpeed(rightmotor, 0.0);


    
    turnrunning = false;
    return 23.55;
}


}//while,end
case 0: 
   turnrunning = true;
float d = 0;
    while(true) {     
    if(d != 0.5) {

    SetSpeed(leftmotor, -0.5);
    SetSpeed(rightmotor, 0.5);
    d = 0.5;
}

        
    if(totaldistance == line) {
    SetSpeed(leftmotor, 0.0);
    SetSpeed(rightmotor, 0.0);
    
    
    turnrunning = false;
    return 35.325;
    
}
}//while,end


}//switch終わり


}//RobotTurn終わり


    
//ここから下はAIにちょっと修正してもらったやつ

#include "Encoder.h"
#include <array>
#include <cmath>

namespace
{

bool xory;                 // x または y の判断用
double x = 0.0;            // x座標
double y = 0.0;            // y座標
double count = 0.0;        // 回転したときの走行距離
bool once = false;         // IncreaseAxisの最初だけ行う処理用
int Kakudo = 0;            // 現在の角度
int deg = 0;               // IncreaseAxisで使う角度
double hypotenuse = 0.0;   // 原点までの距離
double DFH = 0.0;          // atan2で出した角度
int AbsDegree = 0;         // 四捨五入後の角度
int turndeg = 0;          // 原点へ向くための回転角度
int result = 0;           // 結果
double Cx = 0.0;          // 区間開始時のx座標
double Cy = 0.0;          // 区間開始時のy座標
int add = 0;              // 90度を何回分回ったか

// 既存のcallback側などで更新されている前提
extern bool Mrun;
extern double TotalDistance;
extern double xyPa;

int GetDegree_man();
int GetDegree(double turnDistance);
std::array<double, 2> IncreaseAxis(int degrees);
int returnToOrigin(double Xe, double Ye);
int AverageChecker(double dd);

} // namespace


int GetDegree_man()
{
  // 機体の回転が終わったら Mrun が true になる
  if (Mrun) {
    // 回転前の距離との差を使う想定
    count = TotalDistance; //ここに回転始める前の距離を引く
    // 回転距離から何度回ったかを計算
    Kakudo = Kakudo + GetDegree(count);

    // 0～359度に戻す
    while (Kakudo >= 360) {
      Kakudo -= 360;
    }

    Mrun = false;
  }

  return Kakudo;
}


int GetDegree(double turnDistance)
{
  // 例: 23 ÷ 2 = 11.5 なので add = 2
  add = AverageChecker(turnDistance);

  // 次の直進区間では、現在位置を基準にする
  once = true;

  // 2 × 90 = 180度
  return add * 90;
}


std::array<double, 2> IncreaseAxis(int degrees)
{
  // 旋回直後、最初の一回だけ基準位置と基準距離を保存
  if (once) {
    deg = degrees;
    xyPa = TotalDistance;

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
}


int returnToOrigin(double Xe, double Ye)
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
}


int AverageChecker(double dd)
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
}
