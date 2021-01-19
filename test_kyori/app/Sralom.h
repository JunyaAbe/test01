#include "Motor.h"
#include "ColorSensor.h"
#include "SonarSensor.h"
#include "Clock.h"  // <3>
#include "Distance.h"
#include "Direction.h"
#include "util.h"

using namespace ev3api;

/*障害物回避状態変化用*/
enum Move{
  st1,
  tn1,
  st2,
  tn2
};

enum Step{
  step1,
  step2,
  step3
};

#define DELTA_T 0.004         //周期
#define KP 1.80               //現在
#define KI 0.035              //過去↑
#define KD 0.030              //未来↓
#define target 45

class Sralom {
public:
  
  Sralom(Clock C, Distance Dis, Direction Dir);
  void init();
  void run();
  void trace_run();
  float calc_pid_value();
  void turn(const int8_t pwm);
  void stop();
  void terminate();

  Step run_step1();
  Step run_step2();
  Step run_step3();

private:
  Motor leftWheel;
  Motor rightWheel;
  ColorSensor colorSensor;
  SonarSensor sonarSensor;
  Direction direction;
  Clock clock;
  const int8_t mThreshold = 20;
  const int8_t pwm_sralom = (Motor::PWM_MAX) / 10;
};
