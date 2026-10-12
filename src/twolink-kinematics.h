#pragma once
#include <Radian.h>
#include <Vector2D.h>
// both from https://github.com/Tomoooji/mathmatics_tmj

class TwoLink {
private:
  struct Positions {
    Vector2D<float> Base, Joint, End;
  } _points;
  const float _lengthBaseJoint, _lengthJointEnd;
  const bool _isJointUpper;

protected:
  TwoLink(Positions &&points, const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper);

  static bool isReachable(const float lengthBaseJoint, const float lengthJointEnd, Vector2D<float> vectorBaseJoint);
  bool isReachable(Vector2D<float> position) {
    return this->isReachable(this->_lengthBaseJoint, this->_lengthJointEnd, position - this->_points.Base);
  }

  static TwoLink::Positions calcForward(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper,
                                        RadianAbsPi<float> baseAngle,
                                        RadianAbsPi<float> jointAngle,
                                        Vector2D<float> basePosition = Vector2D<float>(0, 0));

  TwoLink::Positions calcForward(RadianAbsPi<float> baseAngle, RadianAbsPi<float> jointAngle) {
    return TwoLink::calcForward(this->_lengthBaseJoint, this->_lengthJointEnd, this->_isJointUpper,
                                baseAngle, jointAngle, this->_points.Base);
  }

  static TwoLink::Positions calcInverse(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper,
                                        Vector2D<float> endPosition,
                                        Vector2D<float> basePosition = Vector2D<float>(0, 0));
                                        
  TwoLink::Positions calcInverse(Vector2D<float> endPosition, Vector2D<float> basePosition = Vector2D<float>(0, 0)) {
    return TwoLink::calcInverse(this->_lengthBaseJoint, this->_lengthJointEnd, this->_isJointUpper,
                                endPosition, basePosition);
  }

public:
  TwoLink(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper,
          RadianAbsPi<float> initAngleBase = 0.0f,
          RadianAbsPi<float> initAngleJoint = 0.0f,
          Vector2D<float> initBasePos = Vector2D<float>(0, 0));
  TwoLink(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper,
          Vector2D<float> initEndPos,
          Vector2D<float> initBasePos = Vector2D<float>(0, 0));
  virtual ~TwoLink() = default;
  
  bool setEndPosition(Vector2D<float> position, bool asDelta = false);
  void setAngles(RadianAbsPi<float> baseAngle, RadianAbsPi<float> jointAngle, bool asDelta = false);
  void setBaseAngle(RadianAbsPi<float> angle, bool asDelta = false) {
    this->setAngles((asDelta ? this->getBaseAngle() + angle : angle), this->getJointAngle(), false);
  }
  void setJointAngle(RadianAbsPi<float> angle, bool asDelta = false) {
    this->setAngles(this->getBaseAngle(), (asDelta ? this->getJointAngle() + angle : angle), false);
  }
  
  bool isJointUpper() const { return this->_isJointUpper; }

  RadianAbsPi<float> getBaseAngle() const { return (this->_points.Joint - this->_points.Base).Theta(); }
  RadianAbsPi<float> getJointAngle() const;
  RadianAbsPi<float> getEndSlope() const;

  const Vector2D<float> &getBase() const { return this->_points.Base; }
  const Vector2D<float> &getJoint() const { return this->_points.Joint; }
  const Vector2D<float> &getEnd() const { return this->_points.End; }

  const float &lengthBaseJoint() const { return this->_lengthBaseJoint; }
  const float &lengthJointEnd() const { return this->_lengthJointEnd; }
};