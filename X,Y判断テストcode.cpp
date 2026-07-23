#include "Enxcoder.h"
#include <cmath>


namespace {

int plus; //a
bool xory; //x or y判断
double  x = 0.0;
double  y = 0.0;
double count = 0.0;
bool once = false;
bool first = false;
int Kakudo = 0;
int* Dptr = nullptr;
double yPa = 0.0;
double xPa = 0.0;
int deg = 0;   
double hypotenuse = 0.0;
double DFH = 0.0;
int AbsDegree = 0;
int turndeg = 0;
int result = 0;
int GetDegree_man();
double GetDegree();
int IncreaseAxis();
double returnToOrigin();
double Cx = 0.0;
double Cy = 0.0;

} //クラス内で定義

int GetDegree_man() {

if(Mrun){

count = TotalDistance;　//これとモーターが回転する前の移動距離の値と引く

Kakudo = Kakudo + GetDegree(count);


Mrun = false;

while(Kakudo >= 360) {
if(Kakudo >= 360) {

Dptr = &Kakudo;
*Dptr -= 360;
}

}//while end

}
if(Kakudo == 180) {
yPa = 0;

}

if(Kakudo == 270) {


  
}

return Kakudo;
} //GetDegree_man,Endほんまはifの中もっとあるけど今書いてない

double GetDegree() {
//ここで90°回ったら進む距離で割る.例えば90°で約100mm進むならint ad = count / 100;

count = 0;
once = true; //increaseAxisで最初にやる処理のため
first = true;

return ad * 90;
} //GetDegree終わり

//ほんでKakudoの値をIncreaseAxis引数に入れて

int IncreaseAxis(int degrees) {

if(once) {
deg = degrees;
once = false;

}//最初だけやること

if(deg == 0) {

xory = true;
y = (TotalDistance - x); 
Cy = (TotalDistance - x); //yノクローン、yをマイナスする仕組みを作るために
xyPa = TotalDistance;

} //0

if(deg == 90) {

xory = false;
x = (TotalDistance - y);
Cx = (TotalDistance - y); //xのクローン
xyPa = TotalDistance;

}//90

if(deg == 180) {

xory = true;
y = Cy - (TotalDistance - xyPa);

} //180

if(deg == 270) {

xory = false;
x = Cx - (TotalDistance - xyPa);

}//270

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
