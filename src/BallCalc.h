#ifndef ballCalc_h
#define ballCalc_h

#include "BallCalc.h"
#include "CircleShift.h"
#include "Complex.h"
#include "softwareFilter.h"

typedef struct{
    float x;
    float y;
} XY;

typedef struct{
    float radius;
    float theta;
} RT;

class BallCalc{
    protected:
        //内部変数
        float* IRAngle;
        float* IRCos;
        float* IRSin;
        float* IRValue;
        softwareFilter<float>** IRData; 
        XY Ball_XY;
        RT Ball_RT;

        //内部定数（変更可）
        int SIZE = 16;
        int SELECT_NUM = 4;
        int LOOP_NUM = 10;
        float REFERENCE = 800, INCLINATION = 200;
        XY REF_XY = {0.0,0.0};
        
        //内部関数
        int selectIR(float Value[]);
        XY CalcXY();
        RT CalcRT(XY XY);
        RT CalcDribblerRT(XY XY);
        float wraparoundConst(float length);
        float wraparoundCalc(RT RT);
    public:
        //実行に必要な関数
        BallCalc(float* Angle, int size = 16, int selectNum = 4, int loopNum = 10);
        ~BallCalc();
        void getIR(float Value[]);
        float calc(bool dribbler);
        
        //内部定数＿変更用関数
        void setRefAngle(float Angle);
        void setWraparound(float reference, float inclination);

        //デバック用関数
        void printIR(float sendData[]);
        void printFilterIR(float sendData[]);
        void printXY(float sendData[]);
        void printRT(float sendData[]);
};

//コンストラクタ　各種変数の初期化
BallCalc::BallCalc(float* Angle, int size, int selectNum, int loopNum)
: IRAngle(Angle), SIZE(size), SELECT_NUM(selectNum), LOOP_NUM(loopNum) {
    //メモリの確保
    IRCos = new float[SIZE];
    IRSin = new float[SIZE];
    IRData = new softwareFilter<float>*[SIZE];
    for(int i=0;i<SIZE;i++){
        IRData[i] = new softwareFilter<float>(LOOP_NUM);
    }
    //各IRセンサのcosとsinを計算
    for(int i=0;i<SIZE;i++){
        IRCos[i] = cos(Angle[i]*PI/180);
        IRSin[i] = sin(Angle[i]*PI/180);
    }
}

//デストラクタ　動的に確保したメモリの解放
BallCalc::~BallCalc() {
    delete[] IRCos;
    delete[] IRSin;
    //クラスのデータ解放がdelete関数で出来なかったので、free関数を使用
    for(int i=0;i<SIZE;i++){
        free(IRData[i]); 
    }
    free(IRData);
}

//各IRセンサの値を取得する
void BallCalc::getIR(float Value[]){
    for(int i=0;i<SIZE;i++){
        IRValue[i] = Value[i];
        IRData[i]->dataAdd(IRValue[i]); 
    }
}

//全体の計算
float BallCalc::calc(bool dribbler = false){
    XY Ball_XY = CalcXY();
    if(dribbler){
        RT Ball_RT = CalcDribblerRT(Ball_XY);
    } else {
        RT Ball_RT = CalcRT(Ball_XY);
    }
    return wraparoundCalc(Ball_RT);
}

//ボールの機体から見た座標を計算
XY BallCalc::CalcXY(){
    XY result = {0.0,0.0};

    float IRFilterValue[SIZE];
    for(int i=0;i<SIZE;i++){
        IRFilterValue[i] = IRData[i]->filter();
    }

    int selectPin = selectIR(IRFilterValue);

    for(int i=0; i < SIZE;i++){
        int pin = CircleShift(SIZE, selectPin, i);
        result.x += IRFilterValue[pin] * IRCos[pin];
        result.y += IRFilterValue[pin] * IRSin[pin];
    }

    return result;
}

//計算に使うIRセンサを選択
int BallCalc::selectIR(float Value[]){
    float maxValue = 0.0;
    int maxPin;
    for(int i=0;i<SIZE;i++){
        if(maxValue < Value[i]){
            maxValue = Value[i];
            maxPin = i;
        }
    }

    int selectPin;
    if(Value[CircleShift(SIZE, maxPin, 1)] < Value[CircleShift(SIZE, maxPin, -1)]){
        selectPin = CircleShift(SIZE,maxPin,-1*SIZE/2);
    } else {
        selectPin = CircleShift(SIZE,maxPin,-1*SIZE/2 + 1);
    }

    return selectPin;
}

//フィールド座標系における機体からボールの距離と角度を計算
RT BallCalc::CalcRT(XY XY){
    Complex<float> Ball_Complex(XY.x, XY.y);
    Complex<float> REF_Complex(REF_XY.x, REF_XY.y);
    RT result;
    result.radius = Complex<float>::absolute(Ball_Complex);
    result.theta = Complex<float>::arg(Ball_Complex / REF_Complex) * 180 / PI;
    return result;
}

//フィールド座標系における機体からボールの距離と角度を計算（ドリブラーを考慮）
RT BallCalc::CalcDribblerRT(XY XY){
    Complex<float> Ball_Complex(XY.x, XY.y);
    Complex<float> REF_Complex(REF_XY.x, REF_XY.y);
    Complex<float> Diff_Complex(0.0,0.0);

    RT result;
    result.radius = Complex<float>::absolute(Ball_Complex);

    Diff_Complex = Ball_Complex / REF_Complex;
    if(Diff_Complex.real >= 0){
        result.theta = Complex<float>::arg(Diff_Complex) * 180 / PI;
    } else {
        Diff_Complex.real *= -1; 
        result.theta = Complex<float>::arg(Diff_Complex) * 180 / PI + 180;
        if(result.theta > 180) result.theta -= 360;
    }
    
    return result;
}

//回り込みを考慮した角度の計算
float BallCalc::wraparoundCalc(RT RT){
    return RT.theta * wraparoundConst(RT.radius);
}

//回り込み定数の計算
float BallCalc::wraparoundConst(float length){
    //回り込み定数の計算　範囲は1～2、基準距離で1.5になるように設定
    return atan2((REFERENCE-length) / INCLINATION, 1.0) / 3.0 + 1.5;
}

void BallCalc::setRefAngle(float Angle){
    REF_XY.x = cos(Angle*PI/180);
    REF_XY.y = sin(Angle*PI/180);
}

void BallCalc::setWraparound(float reference, float inclination){
    REFERENCE = reference;
    INCLINATION = inclination;
}

void BallCalc::printIR(float sendData[]){
    for(int i=0;i<SIZE;i++){
        sendData[i] = IRValue[i];
    }
}

void BallCalc::printFilterIR(float sendData[]){
    for(int i=0;i<SIZE;i++){
        sendData[i] = IRData[i]->filter();
    }
}

void BallCalc::printXY(float sendData[]){
    sendData[0] = Ball_XY.x;
    sendData[1] = Ball_XY.y;
}

void BallCalc::printRT(float sendData[]){
    sendData[0] = Ball_RT.radius;
    sendData[1] = Ball_RT.theta;
}

#endif