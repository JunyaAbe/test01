#ifndef _DISTANCE_H_
#define _DISTANCE_H_

#include "Motor.h"
#include "ev3api.h"
#include "util.h"

using namespace ev3api;

#define TIRE_DIAMETER 90.0  //タイヤ直径（81mm）
#define PI 3.14159265358    //円周率

class Distance{
public:
    //friend class Tracer;
    Distance();
    void init();
    void update();
    float getDistance();
    float getDistance4msRight();
    float getDistance4msLeft();
    float distance;     //走行距離
    float distance4msL; //左タイヤの4ms間の距離
    float distance4msR; //右タイヤの4ms間の距離
    float pre_angleL, pre_angleR; // 左右モータ回転角度の過去値

private:
    Motor leftWheel;
    Motor rightWheel;
};


#endif