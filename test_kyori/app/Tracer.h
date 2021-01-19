#include "Motor.h"
#include "ColorSensor.h"
#include "SonarSensor.h"
#include "util.h"

using namespace ev3api;
#if defined(MAKE_RIGHT)
static const int _LEFT = 0;
#define _EDGE -1
#else
static const int _LEFT = 1;
#define _EDGE 1
#endif
#define DELTA_T 0.004         //周期
#define LIMIT 80        /*PID制御値の上下限*/
class Tracer {
public:
  Tracer();
  void run();
  void init();
  void terminate();
  void acceleration();
  int blue();
  int armup();
  int armdown();
  int halfspeed();
  int turning();
  int blue_cnt;
  int armup_cnt;
  int armdown_cnt;
  int taildown_cnt;
  int turning_cnt;
private:
  Motor leftWheel;
  Motor rightWheel;
  Motor arm;
  Motor tail;
  ColorSensor colorSensor;
  SonarSensor sonarSensor;
  float KP, KI, KD;
  
  const int8_t mThreshold = 20;
  const int8_t pwm = (Motor::PWM_MAX) / 1.2;

  float calc_pid_value(int mode, const int target);
};
