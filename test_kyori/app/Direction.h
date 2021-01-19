#ifndef _DIRECTION_H_
#define _DIRECTION_H_

#include "Motor.h"
#include "ev3api.h"
#include "util.h"

#include "Distance.h"

using namespace ev3api;

#define TREAD 132.6 //車体トレッド幅(13.26cm)
#define PI 3.14159265358    //円周率

class Direction{
public:
    Direction();
    void init();
    void update(); 
    float getDirection();       /* 方位を取得(右旋回が正転) */
    float direction;            //方位
    Distance distance;          //対象の走行距離

private:
    Motor leftWheel;
    Motor rightWheel;
};

#endif