namespace {
constexpr int plus;//a
constexpr bool xory;//x or y判断
constexpr double  x = 0.0;
constexpr double  y = 0.0;
constexpr double count = 0.0;
constexpr bool once = false;
constexpr bool first = false;
} //ヘッダで定義

int GetDegree-man {

if(Mrun/*モーターがtrueを返せばtrueになる変数*/) {
count = TotalDistance - //ここにmotorturnが始まる前の総移動距離;

Kakudo = Kakudo + GetDegree(count);


Mrun = false; //最後にfalseを返す}return Kakudo;
}

}

}//GetDegree-man,End
double GetDegree() {
//ここで90°回ったら進む距離で割る.例えば90°で約100mm進むならint ad = count * 100;  

count = 0;
once = true; //increaseAxisで最初にやる処理のためfirst = true;

return ad * 90;
} //GetDegree終わり

//ほんでKakudoの値をIncreaseAxis引数に入れて

int IncreaseAxis(int degrees) {

if(once) {
int deg = degrees;
once = false;

}//最初だけやること

if(deg = 0) {

xory = true;
y = (TotalDistance - x); 
auto yPa = (TotalDistance - x)

} //0

if(deg = 90) {

xory = false;
x = (TotalDistance - y);
auto xPa = (TotalDistance - y);

}//90

if(deg = 180) {

xory = true;
y -= ((TotalDistance - x) - yPa);

} //180

if(deg = 270) {

xory = true;
x -= ((TotalDistance - y) - xPa);

}//270

}//IncreaseAxis

//もし360を超えた時

while(Kakudo >= 360) {
if(Kakudo >= 360) {

Dptr = &Kakudo;
*Dptr -= 360;
}
if(Kakudo < 360)
break;
} //GetDegree-manの中に入れとく
