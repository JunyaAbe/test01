#include "app.h" // <1>
#include "Tracer.h" // <2>
#include "Clock.h"  // <3>
#include "Distance.h"
#include "Direction.h"
#include "Sralom.h"
#include "util.h"
#include "Garage.h"

#include "TouchSensor.h"

#include <stdio.h>

using namespace ev3api;

#define _SIM 1        //シミュレータである
#define _RIGHTEDGE 1  //右エッジであるかいないか

Clock clock;    // <5>
Tracer tracer;  // <4>
Distance distance_trace,distance_sralom,distance_garage;
Direction direction_sralom;
Section section = linetrace_section;     // 区間指定
//Section section = garage_section;
//Section section = sralom_section;
Step sralom_step = step1;             //スラローム区間内ステップ指定
Garage garage;
Sralom sralom(clock, distance_sralom, direction_sralom);
TouchSensor touchSensor(PORT_1);
static float dist_trace = 0;
static float dist_sralom = 0;
static float dist_garage = 0;
static int tracer_cnt = 0;//50_90
static int tracer_step = 0;
void main_task(intptr_t unused) { // <1>


  _log("start");
  _log("HackEV test_sralom");

  if (_RIGHTEDGE)  _log("Left course:");
  else             _log("Right course:");

  _log("Go to the start, ready?");
  if (_SIM)   _log("Hit SPACE bar to start");
  else        _log("Tap Touch Sensor to start");

 

  /* スタート待機 */
  
  while(1)
  {
      if (touchSensor.isPressed() == true)
      {
            break; // タッチセンサが押された
      }
      clock.sleep(duration_start);    //1000msウェイト
  }

  _log("start");
  tracer.init(); // <3>
  distance_trace.init();
  distance_sralom.init();
  sralom.init();
  direction_sralom.init();
  garage.init();
  
  while(1){
  /*ライントレース走行*/
    if(section == linetrace_section){
      dist_trace = distance_trace.getDistance();
      distance_trace.update();
      //printf("step %d cnt %d ",tracer_step, tracer_cnt);
      if(dist_trace <= 10050){
        if((dist_trace >= 3200 && dist_trace <= 3600)||(dist_trace >= 4200 && dist_trace <= 4700)||(dist_trace >= 6300 && dist_trace <= 6500)||(dist_trace >= 7400 && dist_trace <= 8500)||(dist_trace >= 9300 && dist_trace <= 10050)){
          tracer.acceleration(); //加速
          printf("accel ");
        }else{
          tracer.run();
          //tracer.halfspeed();
        }
      }
      else if(dist_trace >= 10051 && tracer_step == 0)
      {
        tracer_cnt = tracer.blue();
        if(tracer_cnt >= 26){
          tracer_step = 1;
          tracer_cnt = 0;
        };
        //printf("blue");
      }else if(tracer_step == 1)
      {
        tracer_cnt = tracer.armup();
        if(tracer_cnt >= 55){
          tracer_step = 2;
          tracer_cnt = 0;
        }
      }else if(tracer_step == 2)
      {
        tracer_cnt = tracer.armdown();
        if(tracer_cnt >= 25){
          tracer_step = 3;
          tracer_cnt = 0;
        }
      }else if(tracer_step == 3)
      {
        tracer_cnt = tracer.halfspeed();
        if(tracer_cnt >= 95){//165 85
          //section = sralom_section;
          tracer_step = 4;
          tracer_cnt = 0;
          
        }
      }else if(tracer_step == 4)
      {
        tracer_cnt = tracer.turning();
        if(tracer_cnt >= 788){
        section = garage_section;
        tracer_cnt = 0;
        }
      }
      printf("%d  ", tracer_cnt);
      //printf("%f ", dist_trace);
    }
  
    /*スラローム走行*/
    
    else if(section == sralom_section){
      //sectionが切り替わるまで続ける
      //printf("sralom_section");
        switch(sralom_step){
            case step1:                     //スラローム区間内ステップ1
                sralom_step = sralom.run_step1();
                break;
            
            case step2:                     //スラローム区間内ステップ2
                sralom_step = sralom.run_step2();
                //section = garage_section;
                break;
            case step3:                     //スラローム区間内ステップ3
                sralom_step = sralom.run_step3();
                section = garage_section;
                break;

            default:
		        _log("others");
		        break;
        }
        clock.sleep(duration_time);    //40msウェイト
        //if(section != sralom)break;    
    }
    
    /*ガレージ走行*/
    else if(section ==  garage_section){
      int blue_cnt;
      
      if((tracer_cnt <= 90)||(tracer_cnt >= 145 && tracer_cnt <= 260)){//300
        blue_cnt = tracer.halfspeed();
        garage.armstop();
        if(tracer_cnt >= 200 && tracer_cnt <= 220){
          //garage.back();
        }
      }else if(tracer_cnt >= 90 && tracer_cnt <= 130){
        garage.armup();
      }else if(tracer_cnt >= 131 && tracer_cnt <= 144){
        garage.armdown();
        //garage.taildown();
      }
      
                                                                                  
      /*
      if(tracer_cnt <= 80){
        blue_cnt = tracer.halfspeed();
      }else if(tracer_cnt >= 81 && tracer_cnt <= 100){
        blue_cnt = tracer.blue();
      }else if(tracer_cnt >= 101 && tracer_cnt <= 220){
        blue_cnt = tracer.halfspeed();
        
      }*/
      else{
        garage.parking();
      }
      printf("%d \n",tracer_cnt);
      tracer_cnt++;

      //clock.sleep(duration_time);    //40msウェイト
    }
    clock.sleep(duration_time);    //40msウェイト
  }
  _log("end");

/*
  sta_cyc(TRACER_CYC); // <4>
  while (!ev3_button_is_pressed(LEFT_BUTTON)) { // <1>
      clock.sleep(duration);   // <2>
  }
  stp_cyc(TRACER_CYC); // <3>
  tracer.terminate(); // <4>
*/

  ext_tsk(); // <5>
}


void tracer_task(intptr_t exinf) { // <1>

  tracer.run(); // <2>
  distance_trace.update();
  ext_tsk();
}

//*****************************************************************************
// 関数名 : _syslog
// 引数 :   int   level - SYSLOGレベル
//          char* text  - 出力文字列
// 返り値 : なし
// 概要 : SYSLOGレベルlebelのログメッセージtextを出力します。
//        SYSLOGレベルはRFC3164のレベル名をそのまま（ERRだけはERROR）
//        `LOG_WARNING`の様に定数で指定できます。
//*****************************************************************************
static void _syslog(int level, char* text){
    static int _log_line = 0;

    if (_SIM)
    {
        syslog(level, text);
    }
    else
    {
        //ev3_lcd_draw_string(text, 0, CALIB_FONT_HEIGHT*_log_line++);
    }
}

//*****************************************************************************
// 関数名 : _log
// 引数 :   char* text  - 出力文字列
// 返り値 : なし
// 概要 : SYSLOGレベルNOTICEのログメッセージtextを出力します。
//*****************************************************************************
static void _log(char *text){
    _syslog(LOG_NOTICE, text);
}
