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

Triangle::Triangle(float lneAB, float lenBC):
  _lenAB(lneAB),_lenBC(lenBC),_B(0,0,lneAB,0),_C(0,0,lenBC,0){}

bool Triangle::B_type(bool larger){
  bool is_changed = this->_larger_B == larger;
  this->_larger_B = larger;
  return is_changed;
}

Position Triangle::C_xy(float x, float y, bool as_delta){
  this->_C.x = x + (as_delta? this->_C.x: 0);
  this->_C.y = y + (as_delta? this->_C.y: 0);
  if(x || y || as_delta){
    this->_C.XYtoRA();
    this->calc_inverse();
  }
  return this->_C;
}

Position Triangle::C_ra(float r, float a, bool as_delta){
  this->_C.r = r + (as_delta? this->_C.r: 0);
  this->_C.a = a + (as_delta? this->_C.a: 0);
  this->calc_inverse();
  return this->_C;
}

float Triangle::B_ang(float a, bool as_delta){
  this->_angABC = a + (as_delta? this->_angABC: 0);
  this->calc_forward();
  return this->_angABC;
}

bool Triangle::is_in_range(){
  return this->_lenAB - this->_lenBC < this->_C.r && this->_C.r < this->_lenAB + this->_lenBC;
}

void Triangle::calc_forward(){
  this->_C.x = cos((this->_larger_B?-1:1)*(PI-this->_angABC)+this->_B.a) * this->_lenAB + this->_B.x;
  this->_C.y = sin((this->_larger_B?-1:1)*(PI-this->_angABC)+this->_B.a) * this->_lenAB + this->_B.y;
  this->_C.XYtoRA();
}

void Triangle::calc_inverse(){
  this->_angABC = 
}