/* sakuarm.ino
ArduinoUNOで動かす朔アーム
(手首角度維持/個別関節角度調節付き)
input:アナログジョイスティック+ボタン×3
output:(普通の)Servo
*/
#include <Servo.h>
#include "triangle.h"

constexpr uint8_t arm_pin[3] = {};
constexpr uint8_t stick_pin[2] = {};
constexpr uint8_t button_pin[3] = {};

const int arm_xy_speed = 5;
const int arm_ang_speed = 1;
const int stick_ignore_range = 30;

Triangle arm{36,26};
Servo arm_servo[3];

void setup(){
  arm_servo[0].attach(arm_pin[0]);
  arm_servo[1].attach(arm_pin[1]);
  arm_servo[2].attach(arm_pin[2]);
  
  pinMode(stick_pin[0],INPUT);
  pinMode(stick_pin[1],INPUT);

  pinMode(button_pin[0],INPUT_PULLUP);
  pinMode(button_pin[1],INPUT_PULLUP);
  pinMode(button_pin[2],INPUT_PULLUP);

}

void loop(){
  int valX = analogRead(stick_pin[0]);
  if(abs(valX)<=stick_ignore_range) valX = 0;
  int valY = analogRead(stick_pin[1]);
  if(abs(valY)<=stick_ignore_range) valY = 0;
  
  // スティックを動かした分だけアームが移動(離すとその位置で止まる)
  arm.C_xy(
    valX>0?arm_xy_speed:(valX<0?-arm_xy_speed:0),
    valY>0?arm_xy_speed:(valY<0?-arm_xy_speed:0),
    true
  );

  /*
  スティックとアームの位置を対応(離すと元の位置に戻る)
  arm.C_xy(
    origin[0] + convert(valX),
    origin[1] + convert(valY),
    false
  );
  */

  arm_servo[0].write(constrain(arm.A_ang_deg(),0,180));
  arm_servo[1].write(constrain(arm.B_ang_deg(),0,180));
  arm_servo[2].write(constrain(arm.C_ang_deg()-90,0,180));

  arm.A_ang_deg(arm_servo[0].read());
  arm.B_ang_deg(arm_servo[1].read());

}