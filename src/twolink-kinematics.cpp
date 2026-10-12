#include <Arduino.h>
#include "twolink-kinematics.h"

TwoLink::Vectors TwoLink::calcForward(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper, RadianAbsPi<float> baseAngle, RadianAbsPi<float> jointAngle) {
  TwoLink::Vectors points{};
  points.Base = Vector2D<float>::fromXY(0, 0);
  points.Joint = Vector2D<float>::fromPolar(lengthBaseJoint, baseAngle);
  points.End = Vector2D<float>::fromPolar(lengthBaseJoint + lengthJointEnd, baseAngle + jointAngle * (isJointUpper ? 1 : -1));
  return points;
}

TwoLink::Vectors TwoLink::calcInverse(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper, Vector2D<float> endPosition, Vector2D<float> basePosition) {
  TwoLink::Vectors points{};
  points.End = endPosition;
  points.Base = basePosition;
  RadianAbsPi<float> angleJoint_Base_End =
      acos(((sq(points.End - points.Base).getR()) + sq(lengthBaseJoint) - sq(lengthJointEnd)) / (2 * lengthBaseJoint * (points.End - points.Base).getR()));
  points.Joint = Vector2D<float>::fromPolar(lengthBaseJoint, (points.End - points.Base).Theta() + (isJointUpper ? 1 : -1) * angleJoint_Base_End.normalize());
  return points;
}

TwoLink::TwoLink(Vectors &&points, const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper)
  : _points(std::move(points)), _lengthBaseJoint(lengthBaseJoint), _lengthJointEnd(lengthJointEnd), _isJointUpper(isJointUpper) {}

TwoLink::TwoLink(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper, RadianAbsPi<float> initAngleBase, RadianAbsPi<float> initAngleJoint)
  : TwoLink(TwoLink::calcForward(lengthBaseJoint, lengthJointEnd, isJointUpper, initAngleBase, initAngleJoint), lengthBaseJoint, lengthJointEnd, isJointUpper) {}

TwoLink::TwoLink(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper, Vector2D<float> initEndPos, Vector2D<float> initBasePos)
  : TwoLink(TwoLink::calcInverse(lengthBaseJoint, lengthJointEnd, isJointUpper, initEndPos, initBasePos), lengthBaseJoint, lengthJointEnd, isJointUpper) {}

bool TwoLink::isReachable() {
  return this->isReachable(this->_points);
}

bool TwoLink::isReachable(TwoLink::Vectors points) {
  return (points.End - points.Base).getR() <= (this->_lengthBaseJoint + this->_lengthJointEnd) &&
         (points.End - points.Base).getR() >= abs(this->_lengthBaseJoint - this->_lengthJointEnd);
}

bool TwoLink::setEndPosition(Vector2D<float> position, bool asDelta) {
  this->calcInverse((asDelta ? this->_points.End + position : position), this->_points.Base);
  return this->isReachable();
}

bool TwoLink::setBaseAngle(RadianAbsPi<float> angle, bool asDelta) {
  this->calcForward((asDelta ? this->_points.Base.Theta() + angle : angle), this->getJointAngle());
  return this->isReachable();
}

bool TwoLink::setJointAngle(RadianAbsPi<float> angle, bool asDelta) {
  this->calcForward(this->getBaseAngle(), (asDelta ? this->getJointAngle() + angle : angle));
  return this->isReachable();
}

RadianAbsPi<float> TwoLink::getJointAngle() const {
  return this->getEndSlope() + (this->isJointUpper() ? 1 : -1) * (this->_points.Joint - this->_points.Base).Theta();
}

RadianAbsPi<float> TwoLink::getEndSlope() const {
  return (this->_points.End - this->_points.Joint).Theta();
}
