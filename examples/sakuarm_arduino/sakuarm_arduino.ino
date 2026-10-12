/* sakuarm.ino
ArduinoUNOで動かす朔アーム
(手首角度維持/個別関節角度調節付き)
input:アナログジョイスティック
output:(普通の)Servo
*/
#include <Servo.h>
#include "two_link_kinematics.h"

constexpr uint8_t arm_pin[3] = {9,10,11};
constexpr uint8_t stick_pin[2] = {A0,A1};
constexpr uint8_t button_pin = 8;

const int arm_xy_speed = 5;
const int arm_ang_speed = 1;
const int stick_ignore_range = 30;

TwoLink arm{36,26,true};
Servo arm_servo[3];

void setup(){
  pinMode(stick_pin[0],INPUT);
  pinMode(stick_pin[1],INPUT);
  pinMode(button_pin,INPUT_PULLUP);

  arm_servo[0].attach(arm_pin[0]);
  arm_servo[1].attach(arm_pin[1]);
  arm_servo[2].attach(arm_pin[2]);
  
  arm.setEndPosition(10,10);
}

void loop(){
  int valX = analogRead(stick_pin[0]);
  if(abs(valX)<=stick_ignore_range) valX = 0;
  int valY = analogRead(stick_pin[1]);
  if(abs(valY)<=stick_ignore_range) valY = 0;
  
  if(digitalRead(button_pin)){
    // スティック押し込み中にX,Yでそれぞれの関節角度を調節
    arm.setBaseAngle(valX>0?arm_ang_speed:(valX<0?-arm_ang_speed:0),true);
    arm.setJointAngle(valY>0?arm_ang_speed:(valY<0?-arm_ang_speed:0),true);
  }
  else{
    // スティックを動かした分だけアームが移動(離すとその位置で止まる)
    arm.setEndPosition(
      valX>0?arm_xy_speed:(valX<0?-arm_xy_speed:0),
      valY>0?arm_xy_speed:(valY<0?-arm_xy_speed:0),
      true // ここをfalseにするとジョイスティックの絶対位置にアームが追従するようになる
    );
  }

  // サーボに角度を出力
  arm_servo[0].write(constrain(arm.getBaseAngle().deg(),0,180));
  arm_servo[1].write(constrain(arm.getJointAngle().deg(),0,180));
  arm_servo[2].write(constrain(arm.getEndSlope().deg()-90,0,180));

  // サーボの角度(0~180°)をアームに反映
  arm.setBaseAngleDeg(arm_servo[0].read(),false);
  arm.setJointAngleDeg(arm_servo[1].read(),false);
}