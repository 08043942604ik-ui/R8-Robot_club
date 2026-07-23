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

include <array>
#include <cmath>

namespace {

bool once = true;

double x = 0.0;
double y = 0.0;

double Cx = 0.0;
double Cy = 0.0;
double xyPa = 0.0;  // 直進開始時の総走行距離

int Kakudo = 0;     // 現在の向き：0, 90, 180, 270

constexpr double kPi = 3.141592653589793;

// 角度を 0 ～ 359 にそろえる
int NormalizeDegree(int degree) {
  degree %= 360;

  if (degree < 0) {
    degree += 360;
  }

  return degree;
}

// 回転終了時に呼ぶ。
// right_turn=true: 右回転、false: 左回転
int GetDegree_man(bool Mrun, bool right_turn, int turn_degrees) {
  if (!Mrun) {
    return Kakudo;
  }

  if (right_turn) {
    Kakudo += turn_degrees;
  } else {
    Kakudo -= turn_degrees;
  }

  Kakudo = NormalizeDegree(Kakudo);

  // 次の直進開始時に距離の基準を取り直す
  once = true;

  return Kakudo;
}

// TotalDistanceを使って現在のx・yを更新する。
// 0度:+y、90度:+x、180度:-y、270度:-x
std::array<double, 2> IncreaseAxis(double TotalDistance) {
  if (once) {
    Cx = x;
    Cy = y;
    xyPa = TotalDistance;
    once = false;
  }

  const double distance = TotalDistance - xyPa;

  if (Kakudo == 0) {
    y = Cy + distance;

  } else if (Kakudo == 90) {
    x = Cx + distance;

  } else if (Kakudo == 180) {
    y = Cy - distance;

  } else if (Kakudo == 270) {
    x = Cx - distance;
  }

  return {x, y};
}

// 原点へ戻る方向を返す
int returnToOrigin(double Xe, double Ye) {
  const double radian = std::atan2(-Xe, -Ye);
  const double degree = radian * 180.0 / kPi;

  const int rounded_degree =
      static_cast<int>(std::round(degree / 90.0)) * 90;

  return NormalizeDegree(rounded_degree);
}

// 原点までの距離
double DistanceToOrigin(double Xe, double Ye) {
  return std::sqrt(Xe * Xe + Ye * Ye);
}

}  // namespace
