#include <Arduino.h>
#include "triangle.h"

float clip2pi(float ang){
  return (ang>TWO_PI? ang-TWO_PI: (ang<0? ang+TWO_PI: ang));
}

void Position::XYtoRA(){
  this->r = sqrt(sq(this->x)+sq(this->y));
  this->a = clip2pi(atan2(this->y, this->x));
}

void Position::RAtoXY(){
  this->x = this->r * cos(this->a);
  this->y = this->r * sin(this->a);
  this->a = clip2pi(this->a);
}

Triangle::Triangle(float lenAB, float lenBC, float angA, float angB, bool is_B_upper):
  _lenAB(lenAB),_lenBC(lenBC),_angABC(angB),_upper_B(is_B_upper),_B{0,0,lenAB,angA},_C{0,0,lenBC,(is_B_upper?1:-1)*angA+angB}{
    //this->_B.a = clip2pi(angA);
    //this->_B.r = lenAB;
    this->_B.RAtoXY();
    //this->calc_forward();
    this->_C.RAtoXY();
  }

bool Triangle::B_type(bool upper){
  bool changed = this->_upper_B != upper;
  this->_upper_B = upper;
  return changed;
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

float Triangle::A_ang_rad(float a, bool as_delta){
  if(a || !as_delta) this->C_ra((as_delta? a: a-this->_B.a),true);
  return this->_B.a;
}

float Triangle::A_ang_deg(float a, bool as_delta){
  return degrees(this->A_ang_rad(radians(a), as_delta));
}

float Triangle::B_ang_rad(float a, bool as_delta){
  this->_angABC = clip2pi(a + (as_delta? this->_angABC: 0));
  this->calc_forward();
  return this->_angABC;
}

float Triangle::B_ang_deg(float a, bool as_delta){
  return degrees(this->B_ang_rad(radians(a), as_delta));
}

float Triangle::C_ang_rad(){
  return clip2pi((this->_upper_B?1:-1)*this->_B.a + this->_angABC);
}

float Triangle::C_ang_deg(){
  return degrees(this->C_ang_rad());
}

bool Triangle::is_in_range(){
  return (this->_lenAB - this->_lenBC < this->_C.r) && (this->_C.r < this->_lenAB + this->_lenBC);
}

void Triangle::calc_forward(){
  this->_C.x = cos((this->_upper_B?-1:1)*(PI-this->_angABC)+this->_B.a) * this->_lenAB + this->_B.x;
  this->_C.y = sin((this->_upper_B?-1:1)*(PI-this->_angABC)+this->_B.a) * this->_lenAB + this->_B.y;
  this->_C.XYtoRA();
}

void Triangle::calc_inverse(){
  this->_angABC = acos(constrain((sq(this->_lenAB)+sq(this->_lenBC)-sq(this->_C.r)) / (2*this->_lenAB*this->_lenBC+0.001),-1,1));
  this->_B.a = this->_C.a + (this->_upper_B?1:-1)*acos(constrain((sq(this->_lenAB)+sq(this->_C.r)-sq(this->_lenBC)) / (2*this->_lenAB*this->_C.r+0.001),-1,1));
  this->_B.RAtoXY();
}