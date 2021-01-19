#include "Sralom.h"
#include "util.h"
#include <math.h>

using namespace std;

static int diff[2] = {0};             /*差分格納用配列*/
static float integral;            /*積分結果*/

Sralom::Sralom(Clock C, Distance Dis, Direction Dir):
  leftWheel(PORT_C), rightWheel(PORT_B),
  colorSensor(PORT_3),sonarSensor(PORT_2){
      clock = C;
      direction.distance = Dis;
      direction = Dir;
}

void Sralom::init() {
  init_f("Sralom");
}

void Sralom::terminate() {
  msg_f("Stopped.", 1);
  leftWheel.stop();
  rightWheel.stop();
}

void Sralom::run() {
  msg_f("running...", 1);
  leftWheel.setPWM(pwm_sralom);
  rightWheel.setPWM(pwm_sralom);
}

void Sralom::turn(const int8_t pwm){           //pwm>0の時、時計回り（右旋回）
  msg_f("turning...", 1);
  leftWheel.setPWM(pwm);
  rightWheel.setPWM(-pwm);
}

void Sralom::stop(){
  msg_f("stopping...", 1);
  leftWheel.setPWM(0);
  rightWheel.setPWM(0);
}

float Sralom::calc_pid_value(){
  rgb_raw_t tmp;
  float V;	//明度
  float pid;
  float p=0.0,i=0.0,d=0.0;

  diff[0] = diff[1];                
  colorSensor.getRawColor(tmp);
  V = tmp.r;
  if(tmp.g > V){
    V = tmp.g;
  } 
  if(tmp.b > V){
    V = tmp.b;
  }
  V = V * 100 / 255;
  diff[1] = (int)V - target;
  integral += (float)(diff[1] + diff[0]) / 2.0 * DELTA_T;        /*積分の値の計算*/

  p = KP * diff[1];
  i = KI * integral;
  d = KD * (float)(diff[1] - diff[0]) / DELTA_T;
  pid = p + i + d;            /*旋回量計算*/

  if (pid > 100)pid = 100;
  if (pid < -100)pid = -100;

  return pid;                       
}

void Sralom::trace_run(){
  float turn = calc_pid_value();
  int pwm_l = pwm_sralom - turn;     
  int pwm_r = pwm_sralom + turn;     

  leftWheel.setPWM(pwm_l);
  rightWheel.setPWM(pwm_r);

}


/*スラローム区間内ステップ1*/
Step Sralom::run_step1(){
  static Move move = st1;            //分割した回避走行方法
  static int time = 0;              //ソナーセンサ周期用変数
  static int dist[3] = {16,4,4};    //回避前ソナーセンサ接近閾値
  static int dist2[3] = {30,25,8};  //回避前ソナーセンサ回避方向閾値
  static int distline[3] = {60,100,85}; //斜め直進時の移動量計算用
  static int object = 0;            //回避した障害物数
  static int direction_trun;             //回転角
  static float d;                          //st2で直進する距離
  
  static float pre_distance;
  Step rtn;

  direction.distance.update();
  direction.update();

  if(object == 3){
    //規定個数分障害物を回避したら次のステップへ
    direction.distance.init();
    rtn = step2;

  } else if(move == st1){
    run();
    time++;
    //if(time == 10){
    if(time % 10 == 0){
      time = 0;
      printf("直進時:%d\n",sonarSensor.getDistance());
      if(sonarSensor.getDistance() <= dist[object] ){
        //printf("直進時:%d  time:%d ",sonarSensor.getDistance(),time);
        move = tn1;
        stop();
      }
    }
    rtn = step1;
  } else if(move == tn1){
    time++;
    if(time == 10){
      time = 0;
      if(sonarSensor.getDistance() < dist2[object]){
        printf("旋回時:%d\n",sonarSensor.getDistance());
        if(object % 2 == 0){
          turn(-pwm_sralom);
          //turn(pwm_sralom);   //Rコース
        } else if(object % 2 == 1){
          turn(pwm_sralom);
          //turn(-pwm_sralom);   //Rコース
        }
      }
    }

    if(sonarSensor.getDistance() >= dist2[object]){
      stop();
      direction_trun = (int)direction.getDirection();
      printf("第一旋回角度:%d\n",direction_trun);
      
      d = distline[object] / sin(PI*direction_trun/180);

      if(d < 0) d = d*(-1);
      printf("直進距離：d:%f[mm]\n",d);
      pre_distance = direction.distance.getDistance();
      printf("過去位置：%f \n",pre_distance);

      move = st2;
    }
    rtn = step1;
  } else if(move == st2) {
    //printf("走行距離の差 %f[mm]\n",direction.distance.getDistance()-pre_distance);
    if((direction.distance.getDistance() - pre_distance) < d ){
      run();
    } else {
      stop();
      move = tn2;
    }
    rtn = step1;
  } else if(move == tn2) {
    if(direction_trun < 0){
      //左側にいる場合右旋回
      if(direction.getDirection() < -1.0){
        //printf("第二旋回角度%f\n",direction.getDirection());
        turn(pwm_sralom);
      } else {
        stop();
        object++;
        move = st1;
      }

    } else if(direction_trun > 0){
      //右側にいる場合左旋回
      if(direction.getDirection() > 1.0){
        //printf("第二旋回角度%f\n",direction.getDirection());
        turn(-pwm_sralom);
      } else {
        stop();
        object++;
        move = st1;
      }
    }
    rtn = step1;
  }
  return rtn;
}

/*スラローム区間内ステップ2*/
Step Sralom::run_step2(){
  static Move move = st1;            //分割した回避走行方法
  static int time = 0;              //ソナーセンサ周期用変数
  static int dist = 5;   //回避前ソナーセンサ接近閾値
  static int dist2 = 12;  //回避前ソナーセンサ回避方向閾値
  static int direction_trun;             //回転角
  rgb_raw_t tmp;
  int Max;
  Step rtn;

  direction.distance.update();
  direction.update();
  colorSensor.getRawColor(tmp);

  if(move == st1){
    direction.init();
    run();
    time++;
    if(time % 10 == 0){
      time=0;
      printf("直進時:%d\n",sonarSensor.getDistance());
      if(sonarSensor.getDistance() <= dist ){
        move = tn1;
        stop();
      }
    }
    rtn = step2;
  } else if(move == tn1){
    time++;
    if(time == 10){
      time = 0;
    
      if(sonarSensor.getDistance() < dist2 ){
        printf("旋回時:%d\n",sonarSensor.getDistance());
        //Lコースの時右旋回
        turn(pwm_sralom);
        //Rコース時左旋回
        //turn(-pwm_sralom);
      }
    }

    if(sonarSensor.getDistance() >= dist2 ){
      direction_trun = (int)direction.getDirection();
      printf("旋回角度:%d\n",direction_trun);

      move = st2;
    }
    rtn = step2;
  } else if(move == st2) {
    run();

    //カラーセンサが二回目の黒色を検知した場合move = tn2
    //printf("走行距離%f ",direction.distance.getDistance());
    //printf("(R,G,B)=(%d %d %d) \n",tmp.r,tmp.g,tmp.b);
    Max = tmp.r;
    if(Max < tmp.g) Max = tmp.g;
    if(Max < tmp.b) Max = tmp.b;

    if(direction.distance.getDistance() >= 300 && Max < 10){
      //stop();
      //direction,init();
      move = tn2;
    }
   rtn = step2;
  } else if(move == tn2) {
    turn(pwm_sralom);
    rtn = step2;
    
    //L：旋回角の合計が90度のとき旋回終了
    if( direction.getDirection() >= 84){
      stop();
      printf("最終旋回角度 %f\n",direction.getDirection());
      rtn = step3;
    }
    //R：旋回角の合計が-90度の時旋回終了
    /*if( direction.getDirection() <= -84){
      stop();
      printf("最終旋回角度 %f\n",direction.getDirection());
      rtn = step3;
    }*/
  }
  return rtn;
}

/*スラローム区間内ステップ3*/
Step Sralom::run_step3(){
  Step rtn;
  rtn = step3;
  //run();
  /*printf("壁接近時:%d\n",sonarSensor.getDistance());
  if(sonarSensor.getDistance() <= 10){
    stop();
  }*/

  return rtn;
}
