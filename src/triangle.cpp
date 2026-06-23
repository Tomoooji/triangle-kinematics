#include <Arduino.h>
#include "triangle.h"

void Position::XYtoRA(){
  this->r = sqrt((this->x * this->x)+(this->y * this->y));
  this->a = atan2(this->y, this->x);
}

void Position::RAtoXY(){
  this->x = this->r * cos(this->a);
  this->y = this->r * sin(this->a);
}
