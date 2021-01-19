#include "Distance.h"
#include "util.h"

Distance::Distance():
  leftWheel(PORT_C), rightWheel(PORT_B){
}

/* 初期化関数 */
void Distance::init() {
    //各変数の値の初期化
    distance = 0.0;
    //distance = 10600;
    distance4msR = 0.0;
    distance4msL = 0.0;
    //モータ角度の過去値に現在値を代入
    pre_angleL = leftWheel.getCount();
    pre_angleR = rightWheel.getCount();
}

/* 距離更新（4ms間の移動距離を毎回加算している） */
void Distance::update(){
    float cur_angleL = leftWheel.getCount(); //左モータ回転角度の現在値
    float cur_angleR = rightWheel.getCount();//右モータ回転角度の現在値
    float distance4ms = 0.0;        //4msの距離

    // 4ms間の走行距離 = ((円周率 * タイヤの直径) / 360) * (モータ角度過去値　- モータ角度現在値)
    distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
    distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
    distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
    distance += distance4ms; 
    
    //printf("dist:%f.3 dist4ms:%f.3 ",distance,distance4ms);

    //モータの回転角度の過去値を更新
    pre_angleL = cur_angleL;
    pre_angleR = cur_angleR;
}

/* 走行距離を取得 */
float Distance::getDistance(){
    printf("走行距離 %.2f mm\n ", distance);
    return distance;
}

/* 右タイヤの4ms間の距離を取得 */
float Distance::getDistance4msRight(){
    return distance4msR;
}

/* 左タイヤの4ms間の距離を取得 */
float Distance::getDistance4msLeft(){
    return distance4msL;
}