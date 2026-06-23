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

class Triangle{};