#include "Tracer.h"
#include "util.h"
#include "Motor.h"
#include <stdio.h>
#include <algorithm>
static int diff[2] = {0};             /*差分格納用配列*/
static float integral;            /*積分結果*/

Tracer::Tracer():
  leftWheel(PORT_C), rightWheel(PORT_B),
  colorSensor(PORT_3),sonarSensor(PORT_2),arm(PORT_A) ,tail(PORT_D) {
   
}

void Tracer::init() {
  init_f("Tracer");
  blue_cnt = 0;
  armup_cnt = 0;
  armdown_cnt = 0;
  taildown_cnt = 0;
  turning_cnt = 0;
}

void Tracer::terminate() {
  msg_f("Stopped.", 1);
  leftWheel.stop();
  rightWheel.stop();
}

float Tracer::calc_pid_value(int mode, const int target) {

  rgb_raw_t tmp;
  
  double V;	//明度
  float pid;
  float p = 0;
  float i = 0;
  float d = 0;
  static float integral; 
  int max = 0;
  colorSensor.getRawColor(tmp);
  //printf("RGB値(%d,%d,%d) ",tmp.r,tmp.g,tmp.b);
  max = tmp.r;
  if(max < tmp.g){
    max = tmp.g;
  }
  if(max < tmp.b){
    max = tmp.b;
  }
  V = max;
 
  
  if(mode == 0){
    KP = 1.35;         /*P制御係数 1.55*/ 
    KI = 0.022;         /*I制御係数 0.042*/  
    KD = 0.048;        /*D制御係数 0.058*/ 
  }else if(mode == 1)
  {
    KP = 0.90;         /*P制御係数 1.17*/ 
    KI = 0.005;         /*I制御係数 0.018*/  
    KD = 0.005;        /*D制御係数 0.038*/ 
    V = tmp.r;
  }else if(mode == 2){
    KP = 0.95;         /*P制御係数 1.17*/ 
    KI = 0.005;         /*I制御係数 0.018*/  
    KD = 0.010;        /*D制御係数 0.038*/ 
  }else if(mode == 3){
    KP = 0.95;         /*P制御係数 1.17*/ 
    KI = 0.01;         /*I制御係数 0.018*/  
    KD = 0.01;        /*D制御係数 0.038*/ 
    V = tmp.r;
  }
  //printf("%.4f %.4f %.4f",KP, KI, KD);

  V = V*100/255;
  diff[0] = diff[1];             
  //printf("RGB値(%d,%d,%d) ",tmp.r,tmp.g,tmp.b);
  diff[1] = V - target;
  integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;        /*積分の値の計算*/

  p = KP * diff[1];
  i = KI * integral;
  d = KD * (diff[1] - diff[0]) / DELTA_T;
  pid = p + i + d;            /*旋回量計算*/
//printf("pid p%f i%f d%f ",p ,i ,d);

  if(pid > LIMIT){
      pid = LIMIT;
  }else if(pid < -LIMIT){
      pid = -LIMIT;
  }
  else{
      /* 何もしない */
  }
  //printf("V:%.3f ",V);
  //printf("%d %d %d ",tmp.r,tmp.g,tmp.b);

  return pid;                       // <4>
}

void Tracer::run() {
  msg_f("running...", 1);
  float turn = calc_pid_value(0,38);
  int pwm_l = pwm - turn * _EDGE;      // <2>
  int pwm_r = pwm + turn * _EDGE;      // <2>

  //printf("distance:%d \n",sonarSensor.getDistance());

  if(sonarSensor.getDistance() <= 10){
    printf("danger");
    leftWheel.stop();
    rightWheel.stop();
  }

  leftWheel.setPWM(pwm_l);
  rightWheel.setPWM(pwm_r);

  //printf("turn:%.3f ",turn);
}

int Tracer::blue()//ゴール
{
  rgb_raw_t tmp;
  
  msg_f("running...", 1);
  colorSensor.getRawColor(tmp);
  if((tmp.b - tmp.r) >= 50){
      blue_cnt++;
      printf("青色検知‼");
    };

  float turn = calc_pid_value(1, 29); // <1>

  int pwm_l = pwm*0.5 - turn * _EDGE;     // <2>
  
  int pwm_r = pwm*0.5 + turn * _EDGE;      // <2>

  
  //printf("PID値%.1f\n", turn);

  leftWheel.setPWM(pwm_l);
  rightWheel.setPWM(pwm_r);
  return blue_cnt;
}
int Tracer::armup(){ 
  msg_f("running...", 1);
  arm.setPWM(10);
  if(armup_cnt >= 45){
    arm.stop();
  }
  
  leftWheel.setPWM(35);
  rightWheel.setPWM(35);
  armup_cnt++;
  return armup_cnt;
}
int Tracer::armdown()
{
  arm.setPWM(-10);
  tail.setPWM(15);
  if(armdown_cnt >= 8){
    arm.stop();
    //tail.stop();
  }

  /*float turn = calc_pid_value(3, 25); // 0, 30
  int pwm_l = pwm/4 - turn;     // <2>
  
  int pwm_r = pwm/4 + turn;      // <2>
  */
  leftWheel.setPWM(25);
  rightWheel.setPWM(25);
  armdown_cnt++;
  return armdown_cnt;
}
int Tracer::halfspeed(){
  msg_f("running...", 1);
  //material = sonarsensor.getDistance();
  //printf("ソナー%d  ",material);
  tail.setPWM(-25);
  float turn = calc_pid_value(3, 25); // 0, 30
  int pwm_l = pwm/6 - turn * _EDGE;     // <2>
  
  int pwm_r = pwm/6 + turn * _EDGE;      // <2>

  
  //printf("PID値%.1f\n",turn);
  
  leftWheel.setPWM(pwm_l);
  rightWheel.setPWM(pwm_r);
  taildown_cnt++;
  return taildown_cnt;
}
int Tracer::turning(){
  msg_f("running...", 1);

  if((turning_cnt <= 21)||(turning_cnt >= 315 && turning_cnt <= 341)||(turning_cnt >= 365 && turning_cnt <= 385)||(turning_cnt >= 408 && turning_cnt <= 412)){
    leftWheel.setPWM(-10 * _EDGE);
    rightWheel.setPWM(10 * _EDGE);
  }else if((turning_cnt >= 95 && turning_cnt <= 116)||(turning_cnt >= 192 && turning_cnt <= 218)||(turning_cnt >= 495 && turning_cnt <= 505)||(turning_cnt >= 536 && turning_cnt <= 550)||(turning_cnt >= 568 && turning_cnt <= 580)||(turning_cnt >= 600 && turning_cnt <= 620)||(turning_cnt >= 755 && turning_cnt <= 765)||(turning_cnt >= 783 && turning_cnt <= 787)){
    leftWheel.setPWM(10 * _EDGE);
    rightWheel.setPWM(-10 * _EDGE);
  }else{
    leftWheel.setPWM(10);
    rightWheel.setPWM(10);
  }
  turning_cnt++;
  return turning_cnt;
}
void Tracer::acceleration() //加速
{
  msg_f("running...", 1);
  float turn = calc_pid_value(0, 40); // <1>

  int pwm_l = (pwm * 1.2) - turn * _EDGE;     // <2>
  
  int pwm_r = (pwm * 1.2) + turn * _EDGE;      // <2>
  //printf("PID値%0.1f\n",turn);
  leftWheel.setPWM(pwm_l);
  rightWheel.setPWM(pwm_r);

}