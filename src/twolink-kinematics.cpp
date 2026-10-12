#include <Arduino.h>
#include "twolink-kinematics.h"

TwoLink::Positions TwoLink::calcForward(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper,
                                        RadianAbsPi<float> baseAngle, RadianAbsPi<float> jointAngle, Vector2D<float> basePosition) {
  TwoLink::Positions points{
    .Base = basePosition,
    .Joint = Vector2D<float>::fromPolar(lengthBaseJoint, baseAngle) + basePosition,
    .End = Vector2D<float>::fromPolar(lengthBaseJoint + lengthJointEnd, baseAngle + jointAngle * (isJointUpper ? 1 : -1)) + basePosition
  };
  return points;
}

TwoLink::Positions TwoLink::calcInverse(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper,
                                        Vector2D<float> endPosition, Vector2D<float> basePosition) {
  TwoLink::Positions points{
    .Base = basePosition,
    .Joint = Vector2D<float>(0, 0),
    .End = endPosition
  };
  RadianAbsPi<float> angle_JointBaseEnd =
      acos(((sq(points.End - points.Base).getR()) + sq(lengthBaseJoint) - sq(lengthJointEnd)) / (2 * lengthBaseJoint * (points.End - points.Base).getR()));
  points.Joint = Vector2D<float>::fromPolar(lengthBaseJoint, (points.End - points.Base).Theta() + (isJointUpper ? 1 : -1) * angle_JointBaseEnd.normalize());
  return points;
}

TwoLink::TwoLink(TwoLink::Positions &&points, const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper)
  : _points(std::move(points)), _lengthBaseJoint(lengthBaseJoint), _lengthJointEnd(lengthJointEnd), _isJointUpper(isJointUpper) {}

TwoLink::TwoLink(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper, RadianAbsPi<float> initAngleBase, RadianAbsPi<float> initAngleJoint, Vector2D<float> initBasePos)
  : TwoLink(TwoLink::calcForward(lengthBaseJoint, lengthJointEnd, isJointUpper, initAngleBase, initAngleJoint, initBasePos), lengthBaseJoint, lengthJointEnd, isJointUpper) {}

TwoLink::TwoLink(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper, Vector2D<float> initEndPos, Vector2D<float> initBasePos)
  : TwoLink(TwoLink::calcInverse(lengthBaseJoint, lengthJointEnd, isJointUpper, initEndPos, initBasePos), lengthBaseJoint, lengthJointEnd, isJointUpper) {}

bool TwoLink::isReachable(TwoLink::Vectors points) {
  return (points.End - points.Base).getR() <= (this->_lengthBaseJoint + this->_lengthJointEnd) &&
         (points.End - points.Base).getR() >= abs(this->_lengthBaseJoint - this->_lengthJointEnd);
}

bool TwoLink::setEndPosition(Vector2D<float> position, bool asDelta) {
  TwoLink::Positions points;
  points = this->calcInverse((asDelta ? this->_points.End + position : position), this->_points.Base);
  if (this->isReachable(points)) {
    this->_points = std::move(points);
    return true;
  } else {
    return false;
  }
}

bool TwoLink::setBaseAngle(RadianAbsPi<float> angle, bool asDelta) {
  TwoLink::Positions points = this->calcForward((asDelta ? this->getBaseAngle() + angle : angle), this->getJointAngle());
  if (this->isReachable(points)) {
    this->_points = std::move(points);
    return true;
  } else {
    return false;
  }
}

bool TwoLink::setJointAngle(RadianAbsPi<float> angle, bool asDelta) {
  TwoLink::Positions points = this->calcForward(this->getBaseAngle(), (asDelta ? this->getJointAngle() + angle : angle));
  if (this->isReachable(points)) {
    this->_points = std::move(points);
    return true;
  } else {
    return false;
  }
}

RadianAbsPi<float> TwoLink::getJointAngle() const {
  return this->getEndSlope() + (this->isJointUpper() ? 1 : -1) * (this->_points.Joint - this->_points.Base).Theta();
}

RadianAbsPi<float> TwoLink::getEndSlope() const {
  return (this->_points.End - this->_points.Joint).Theta();
}
