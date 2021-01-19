#include "Garage.h"
#include "util.h"
#include "Motor.h"
#include <stdio.h>
#include <algorithm>


Garage::Garage():
    leftWheel(PORT_C), rightWheel(PORT_B),
    sonarSensor(PORT_2), arm(PORT_A), tail(PORT_D){
   
}

void Garage::init() {
    init_f("Garage");
}

void Garage::stop(){
    msg_f("stopping...", 1);
    leftWheel.setPWM(0);
    rightWheel.setPWM(0);
}
void Garage::armup(){
    arm.setPWM(10);
    leftWheel.setPWM(15);
    rightWheel.setPWM(15);
}
void Garage::armdown(){
    arm.setPWM(-10);
    leftWheel.setPWM(10);
    rightWheel.setPWM(10);
}
void Garage::back(){
    leftWheel.setPWM(-20);
    rightWheel.setPWM(-20);
}
void Garage::taildown(){
    tail.setPWM(15);
}
void Garage::armstop(){
    arm.setPWM(0);
}
void Garage::parking(){
    static int dist_park = 7;
    if(sonarSensor.getDistance() <= dist_park ){
        stop();
      }else{
          leftWheel.setPWM(15);
          rightWheel.setPWM(15);
      }
}