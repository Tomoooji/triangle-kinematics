#pragma once
#include <Radian.h>
#include <Vector2D.h>
// both from https://github.com/Tomoooji/mathmatics_tmj

class TwoLink {
private:
  //Vector2D<float> Base, Joint, End;
  struct Vectors{ Vector2D<float> Base, Joint, End;} _points;
  const float _lengthBaseJoint, _lengthJointEnd;
  const bool _isJointUpper;

public:
  static TwoLink::Vectors calcForward(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper, RadianAbsPi<float> baseAngle, RadianAbsPi<float> jointAngle);
  static TwoLink::Vectors calcInverse(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper, Vector2D<float> endPosition, Vector2D<float> basePosition = Vector2D<float>(0, 0));
  TwoLink(Vectors &&points, const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper);
  TwoLink(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper = true, RadianAbsPi<float> initAngleBase = 0.0f, RadianAbsPi<float> initAngleJoint = 0.0f);
  TwoLink(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper, Vector2D<float> initEndPos, Vector2D<float> initBasePos = Vector2D<float>(0, 0));
  void calcForward(RadianAbsPi<float> baseAngle, RadianAbsPi<float> jointAngle) { this->_points = TwoLink::calcForward(this->_lengthBaseJoint, this->_lengthJointEnd, this->_isJointUpper, baseAngle, jointAngle); }
  void calcInverse(Vector2D<float> endPosition, Vector2D<float> basePosition = Vector2D<float>(0, 0)) { this->_points = TwoLink::calcInverse(this->_lengthBaseJoint, this->_lengthJointEnd, this->_isJointUpper, endPosition, basePosition); }
  bool isReachable();
  bool isReachable(TwoLink::Vectors points);
  bool setEndPosition(Vector2D<float> position, bool asDelta = false);
  bool setBaseAngle(RadianAbsPi<float> angle, bool asDelta = false);
  bool setJointAngle(RadianAbsPi<float> angle, bool asDelta = false);
  RadianAbsPi<float> getBaseAngle() const { return (this->_points.Joint - this->_points.Base).Theta(); }
  RadianAbsPi<float> getJointAngle() const;
  RadianAbsPi<float> getEndSlope() const;
  bool isJointUpper() const { return this->_isJointUpper; }
  const Vector2D<float> &getBase() const { return this->_points.Base; }
  const Vector2D<float> &getJoint() const { return this->_points.Joint; }
  const Vector2D<float> &getEnd() const { return this->_points.End; }
  const float &lengthBaseJoint() const { return this->_lengthBaseJoint; }
  const float &lengthJointEnd() const { return this->_lengthJointEnd; }
}