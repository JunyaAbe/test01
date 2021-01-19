#include "Direction.h"
#include "util.h"

Direction::Direction():
  leftWheel(PORT_C), rightWheel(PORT_B){
}

void Direction::init(){
  direction = 0.0;
}

void Direction::update(){
  //(360 / (2 * 円周率 * 車体トレッド幅)) * (右進行距離 - 左進行距離)
  direction += (360.0 / (2.0 * PI * TREAD)) * (distance.getDistance4msLeft() - distance.getDistance4msRight());

}

float Direction::getDirection(){
  return direction;
}