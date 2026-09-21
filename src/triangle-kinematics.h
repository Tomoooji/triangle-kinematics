#pragma once
#include <Radian.h>
#include <Vector2D.h>
// both from https://github.com/Tomoooji/mathmatics_tmj

class Triangle {
private:
  Vector2D<float> Base, Joint, End;
  const float _lengthBaseJoint, _lengthJointEnd;
  const bool _isJointUpper;
  bool _isCalculated = false;

public:
  Triangle(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper = true,
           RadianAbsPi<float> initAngleBase = 0.0f, RadianAbsPi<float> initAngleJoint = 0.0f);
  void calcForward();
  void calcInverse();
  bool isReachable();
  bool setEndPosition(Vector2D<float> position, bool asDelta = false);
  bool setBaseAngle(RadianAbsPi<float> angle, bool asDelta = false);
  bool setJointAngle(RadianAbsPi<float> angle, bool asDelta = false);
  RadianAbsPi<float> getJointAngle() const;
  RadianAbsPi<float> getEndSlope() const;
  bool isJointUpper() const { return this->_isJointUpper; }
  const Vector2D<float> &getBase() const { return this->Base; }
  const Vector2D<float> &getJoint() const { return this->Joint; }
  const Vector2D<float> &getEnd() const { return this->End; }
  const float &lengthBaseJoint() const { return this->_lengthBaseJoint; }
  const float &lengthJointEnd() const { return this->_lengthJointEnd; }
}