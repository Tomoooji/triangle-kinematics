// triangle-kinematic class
#pragma once

struct Position{
  float x;
  float y;
  float r;
  float a;
  void XYtoRA();
  void RAtoXY();
};

class Triangle{
private:
  Position _B;
  Position _C;
public:
  Triangle(float lenAB, float lenBC);
  Position updateCxy(float x, float y, bool as_delta);
  Position updateCra(float r, float a, bool as_delta);
  float updateBang(float a, bool as_delta);
  float updateAang(float a, bool as_delta);
  bool setBtype(bool at_upper);
};