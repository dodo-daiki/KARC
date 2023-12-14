#include <KARC.h>
#define IR_NUM 16

int IRPins[IR_NUM] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};


//機体定数
float IRAngle[IR_NUM] = {0,22.5,45,67.5,90,112.5,135,157.5,180,202.5,225,247.5,270,292.5,315,337.5};

//回り込み定数の設定
float REFERENCE = 1000;
float INCLINATION = 250;

//BallCalcクラスの実体化
BallCalc Ball(IRAngle);


//ジャイロセンサの値を取得する関数（ユーザー側で記述）
float readGyro(){
  int Gyro = 0;
  return Gyro;
}

//IRセンサの値を取得する関数（ユーザー側で記述）
float readIR(int pin){
  return digitalRead(pin);
}

void setup() {
  Serial.begin(9600);

  //内部の回り込み定数を変更(デフォルトは(800,200))
  Ball.setWraparound(REFERENCE,INCLINATION);
}

void loop() {
  float IRValue[IR_NUM];
  float moveAngle = 0;
  float BallRT[2];

  for(int i = 0;i<IR_NUM;i++){
    IRValue[i] = readIR(IRPins[i]);
  }

  float gyro = readGyro();
  //クラス内部にIRセンサの値を送信
  Ball.getIR(IRValue);
  //基準角の変更（機体座標とフィールド座標の角度のずれ）
  Ball.setRefAngle(gyro);
  //回り込み角度を計算
  moveAngle = Ball.calc();

  //ボールの角度と距離を取得
  Ball.getBallRT(BallRT);

  Serial.print("radius: ");
  Serial.print(BallRT[0]);
  Serial.print(" theta: ");
  Serial.println(BallRT[1]);
}

