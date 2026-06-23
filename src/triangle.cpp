#include <Arduino.h>
#include "triangle.h"

void Position::XYtoRA(){
  this->r = sqrt(sq(this->x)+sq(this->y));
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

bool Triangle::C_xy(float x, float y, bool as_delta){
  float lastx = this->_C.x;
  float lasty = this->_C.y;
  this->_C.x = x + (as_delta? this->_C.x: 0);
  this->_C.y = y + (as_delta? this->_C.y: 0);
  this->_C.XYtoRA();
  if(this->is_in_range()){
    this->calc_inverse();
    return true;
  }
  else{
    this->_C.x = lastx;
    this->_C.y = lasty;
    this->_C.XYtoRA();
    return false;
  }
}

bool Triangle::C_ra(float r, float a, bool as_delta){
  float lastr = this->_C.r;
  float lasta = this->_C.a;
  this->_C.r = r + (as_delta? this->_C.r: 0);
  this->_C.a = a + (as_delta? this->_C.a: 0);
  this->_C.RAtoXY();
  if(this->is_in_range()){
    this->calc_inverse();
    return true;
  }
  else{
    this->_C.r = lastr;
    this->_C.a = lasta;
    this->_C.RAtoXY();
    return false;
  }
}

float Triangle::B_ang_rad(float a, bool as_delta){
  this->_angABC = a + (as_delta? this->_angABC: 0);
  this->calc_forward();
  return this->_angABC;
}

float Triangle::B_ang_deg(float a, bool as_delta){
  return rad_to_deg(this->B_ang_rad(deg_to_rad(a), as_delta));
}

float Triangle::C_ang_rad(){
  return (this->_larger_B?1:-1)*this->_B.a + this->_angABC;
}

float Triangle::C_ang_deg(){
  return rad_to_deg(this->C_ang_rad());
}

bool Triangle::is_in_range(){
  return (this->_lenAB - this->_lenBC < this->_C.r) && (this->_C.r < this->_lenAB + this->_lenBC);
}

void Triangle::calc_forward(){
  this->_C.x = cos((this->_larger_B?-1:1)*(PI-this->_angABC)+this->_B.a) * this->_lenAB + this->_B.x;
  this->_C.y = sin((this->_larger_B?-1:1)*(PI-this->_angABC)+this->_B.a) * this->_lenAB + this->_B.y;
  this->_C.XYtoRA();
}

void Triangle::calc_inverse(){
  this->_angABC = acos((sq(this->_lenAB)+sq(this->_lenBC)-sq(this->_C.r)) / (2*this->_lenAB*this->_lenBC));
  this->_B.a = this->_C.a + (this->_larger_B?1:-1)*acos((sq(this->_lenAB)+sq(this->_C.r)-sq(this->_lenBC)) / (2*this->_lenAB*this->_C.r));
  this->_B.RAtoXY();
}