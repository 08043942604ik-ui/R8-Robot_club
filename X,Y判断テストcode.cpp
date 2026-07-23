#include "Enxcoder.h"
#include <cmath>


namespace {

bool xory; //x or y判断
double  x = 0.0;　//x座標
double  y = 0.0; //y座標
double count = 0.0; //走行距離
bool once = false; //IncreaseAxisの最初だけ処理したいやつをやるため
int Kakudo = 0; //どのくらいの角度回ったか
int* Dptr = nullptr; 
int deg = 0; //degreeを短くしたやつ
double hypotenuse = 0.0; //斜辺の長さ
double DFH = 0.0; //θ
int AbsDegree = 0; //θの値を四捨五入した値
int turndeg = 0; //ロボットが回転する角度
int result = 0; //結果
int GetDegree_man(); //角度を送る関数
double GetDegree(); //エンコーダーの回ってる時の走行距離を使って角度を求める関数
int IncreaseAxis(); //ロボットの位置のx,yの座標を送る関数
double returnToOrigin(); //原点に帰るための値を求めて送る関数
double Cx = 0.0; 
double Cy = 0.0;
int add = 0; //90にかける値
double AChecker(); //addの値を決める関数


} //クラス内で定義

int GetDegree_man() {

if(Mrun/*これは機体が回ったときにtrueになる*/){

count = TotalDistance;　//これとモーターが回転する前の移動距離の値と引く

Kakudo = Kakudo + GetDegree(count);


Mrun = false;

while(Kakudo >= 360) {
if(Kakudo >= 360) {

Dptr = &Kakudo;
*Dptr -= 360;
}

}//これはKakudoは溜まってくから360を超えたらリセット


}

return Kakudo;
} //GetDegree_man,Endほんまはifの中もっとあるけど今書いてない

double GetDegree() {
add = AChecker();

count = 0;
once = true; //increaseAxisで最初にやる処理のため

return add * 90;
} //GetDegree終わり

//ほんでKakudoの値をIncreaseAxis引数に入れて

int IncreaseAxis(int degrees) {

if(once) {
deg = degrees;
xyPa = TotalDistance;

Cx = x;
Cy = y;
  
once = false;
}//最初だけやること

Rightcase {
if(deg == 0) {

xory = true;
y = Cy + (TotalDistance - xyPa); //xyPaを入れることでカウントが0からになる

} //0

else if(deg == 90) {

xory = false;
x = Cx + (TotalDistance - xyPa);
}//90

else if(deg == 180) {

xory = true;
y = Cy - (TotalDistance - xyPa);
  
} //180

else if(deg == 270) {

xory = false;
x = Cx - (TotalDistance - xyPa);

}//270

}//右回りの場合

leftcase {
if(deg == 0) {

xory = true;
y = (TotalDistance - x); 
Cy = (TotalDistance - x); //yノクローン、yをマイナスする仕組みを作るために
xyPa = TotalDistance;

} //0

else if(deg == 90) {

xory = false;
x = Cx - (TotalDistance - xyPa);

xyPa = TotalDistance;
}//90

else if(deg == 180) {

xory = true;
y = Cy - (TotalDistance - xyPa);
xyPa = TotalDistance;
  
} //180

else if(deg == 270) {

xory = false;
x = (TotalDistance - y);
Cx = (TotalDistance - y); //xのクローン
xyPa = TotalDistance;
  
}//270

}//左回りの場合

return {x, y};//あっち側でXoC = XoC + coordinate[1];みたいなことをする  
}//IncreaseAxis

double returnToOrigin(double Xe, double Ye) {

hypotenuse = std::sqrt((Xe * Xe) + (Ye * Ye)); //三平方の定理で斜辺の長さを求める
DFH = atan2(Ye, Xe);//θを求める

AbsDegree = std::round(DFH);//四捨五入する

turndeg = 180 - (90 + AbsDegree); //まだθと90を足すことで角度を求める

if(Xe < 0) {
result = 360 - turndeg;

return result;
}

if(Xe > 0) {


return result;
}

}//retunrToOrigin終わり

double AverageChecker(double dd) {
for(int g = 1; g < 4; g++) {
int k = 0;

k = dd / g;

if(g = 1 && 11 <= k <= 12) {

return 1;
}

else if(g = 2 && 11 <= k <= 12) {

return 2;
}

else if(g = 3 && 11 <= k <= 12) {

return 3;
}

}//for,end

}//AverageChecker,end





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
