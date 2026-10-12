#include <Arduino.h>
#include "twolink-kinematics.h"

bool TwoLink::isReachable(const float lengthBaseJoint, const float lengthJointEnd, Vector2D<float> vectorBaseJoint) {
  return vectorBaseJoint.getR() <= (lengthBaseJoint + lengthJointEnd) &&
         vectorBaseJoint.getR() >= abs(lengthBaseJoint - lengthJointEnd);
}

TwoLink::Positions TwoLink::calcForward(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper,
                                        RadianAbsPi<float> baseAngle, RadianAbsPi<float> jointAngle, Vector2D<float> basePosition) {
  TwoLink::Positions points{
    .Base = basePosition,
    .Joint = Vector2D<float>::fromPolar(lengthBaseJoint, baseAngle) + basePosition,
    .End = Vector2D<float>::fromPolar(lengthBaseJoint, baseAngle) + basePosition
  };
  points.End += Vector2D<float>::fromPolar(lengthJointEnd, (PI_FOR_RADIAN - jointAngle + (isJointUpper ? 1 : -1) * baseAngle).normalize());
  return points;
}

TwoLink::Positions TwoLink::calcInverse(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper,
                                        Vector2D<float> endPosition, Vector2D<float> basePosition) {
  TwoLink::Positions points{
    .Base = basePosition,
    .Joint = basePosition,
    .End = endPosition
  };
  RadianAbsPi<float> angle_JointBaseEnd = 
      acos(constrain(((sq((points.End - points.Base).getR())) + sq(lengthBaseJoint) - sq(lengthJointEnd)) / (2 * lengthBaseJoint * (points.End - points.Base).getR() + 1e-6f), -1.0f, 1.0f));
  points.Joint += Vector2D<float>::fromPolar(lengthBaseJoint, (points.End - points.Base).Theta() + (isJointUpper ? 1 : -1) * angle_JointBaseEnd.normalize());
  return points;
}

TwoLink::TwoLink(TwoLink::Positions &&points, const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper)
  : _points(std::move(points)), _lengthBaseJoint(lengthBaseJoint), _lengthJointEnd(lengthJointEnd), _isJointUpper(isJointUpper) {}

TwoLink::TwoLink(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper,
                 RadianAbsPi<float> initAngleBase, RadianAbsPi<float> initAngleJoint, Vector2D<float> initBasePos)
  : TwoLink(TwoLink::calcForward(lengthBaseJoint, lengthJointEnd, isJointUpper, initAngleBase, initAngleJoint, initBasePos), lengthBaseJoint, lengthJointEnd, isJointUpper) {}

TwoLink::TwoLink(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper, Vector2D<float> initEndPos, Vector2D<float> initBasePos)
  : TwoLink((TwoLink::isReachable(lengthBaseJoint, lengthJointEnd, initEndPos - initBasePos) ?
            TwoLink::calcInverse(lengthBaseJoint, lengthJointEnd, isJointUpper, initEndPos, initBasePos) : 
            TwoLink::calcForward(lengthBaseJoint, lengthJointEnd, isJointUpper, 0.0f, 0.0f, initBasePos)),
            lengthBaseJoint, lengthJointEnd, isJointUpper) {}

bool TwoLink::setEndPosition(Vector2D<float> position, bool asDelta) {
  Vector2D<float> endPosition = asDelta ? this->_points.End + position : position;
  if (this->isReachable(endPosition)) {
    this->_points = this->calcInverse(endPosition, this->_points.Base);
    return true;
  } else {
    return false;
  }
}

void TwoLink::setAngles(RadianAbsPi<float> baseAngle, RadianAbsPi<float> jointAngle, bool asDelta) {
  this->_points = this->calcForward(
    (asDelta ? this->getBaseAngle() + baseAngle : baseAngle),
    (asDelta ? this->getJointAngle() + jointAngle : jointAngle)
  );
}

RadianAbsPi<float> TwoLink::getJointAngle() const {
  return (PI_FOR_RADIAN - this->getEndSlope() + (this->isJointUpper() ? 1 : -1) * (this->_points.Joint - this->_points.Base).Theta()).normalize();
}

RadianAbsPi<float> TwoLink::getEndSlope() const {
  return (this->_points.End - this->_points.Joint).Theta();
}
