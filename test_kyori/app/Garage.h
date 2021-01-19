#include "Motor.h"
#include "ColorSensor.h"
#include "SonarSensor.h"
#include "Distance.h"

using namespace ev3api;

class Garage{
public:
    Garage();
    void init();
    void armup();
    void armdown();
    void taildown();
    void stop();
    void parking();
    void armstop();
    void back();
private:
    Motor leftWheel;
    Motor rightWheel;
    SonarSensor sonarSensor;
    Motor arm;
    Motor tail;

};