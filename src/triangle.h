// triangle-kinematic class
#pragma once

float clip2pi(float ang);

struct Position{
  float x;
  float y;
  float r;
  float a;
  Position();//float val1 = 0, float val2 = 0, bool as_XY = true);
  void XYtoRA();
  void RAtoXY();
};

class Triangle{
private:
  Position _B;
  Position _C;
  bool _upper_B;
  float _lenAB;
  float _lenBC;
  float _angABC;
  void calc_forward();
  void calc_inverse();
public:
  Triangle(float lenAB, float lenBC, float angA = 0, float angB = 0, bool is_B_upper = true);
  bool B_type(bool larger);
  bool is_in_range();
  bool C_xy(float x = 0, float y = 0, bool as_delta = false);
  bool C_ra(float r = 0, float a = 0, bool as_delta = false);
  float A_ang_rad(float a = 0, bool as_delta = false);
  float A_ang_deg(float a = 0, bool as_delta = false);
  float B_ang_rad(float a = 0, bool as_delta = false);
  float B_ang_deg(float a = 0, bool as_delta = false);
  float C_ang_rad();
  float C_ang_deg();
};