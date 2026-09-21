#include "triangle-kinematics.h"

Triangle::Triangle(const float lengthBaseJoint, const float lengthJointEnd, bool isJointUpper,
                   RadianAbsPi<float> initAngleBase, RadianAbsPi<float> initAngleJoint)
    : _isJointUpper(isJointUpper), _lengthBaseJoint(lengthBaseJoint), _lengthJointEnd(lengthJointEnd),
      Base{Vector2D<float>::fromXY(0, 0)},  Joint{Vector2D<float>::fromPolar(lengthBaseJoint, initAngleBase)},
      End{Vector2D<float>::fromPolar(lengthBaseJoint + lengthJointEnd, initAngleBase + (isJointUpper ? 1 : -1) * initAngleJoint)} {}

void Triangle::calcForward() {
  this->Joint = Vector2D<float>::fromPolar(this->_lengthBaseJoint, this->Base.getTheta());
  this->End = Vector2D<float>::fromPolar(this->_lengthBaseJoint + this->_lengthJointEnd,
      this->Base.getTheta() + (this->_isJointUpper ? 1 : -1) * this->getJointAngle());
  this->_isCalculated = true;
}

void Triangle::calcInverse() {
  RadianAbsPi<float> angleJoint_Base_End =
      acos(((sq(this->End - this->Base).getR()) + sq(this->_lengthBaseJoint) - sq(this->_lengthJointEnd))
       / (2 * this->_lengthBaseJoint * (this->End - this->Base).getR()));
  this->Joint = Vector2D<float>::fromPolar(this->_lengthBaseJoint, (this->End - this->Base).Theta() + (this->_isJointUpper ? 1 : -1) * angleJoint_Base_End.normalize());
  this->_isCalculated = true;
}

bool Triangle::isReachable() {
  return (this->End - this->Base).getR() <= (this->_lengthBaseJoint + this->_lengthJointEnd) &&
         (this->End - this->Base).getR() >= abs(this->_lengthBaseJoint - this->_lengthJointEnd);
}

bool Triangle::setEndPosition(Vector2D<float> position, bool asDelta) {}

bool Triangle::setBaseAngle(RadianAbsPi<float> angle, bool asDelta) {}

bool Triangle::setJointAngle(RadianAbsPi<float> angle, bool asDelta) {}

RadianAbsPi<float> Triangle::getJointAngle() const {}

RadianAbsPi<float> Triangle::getEndSlope() const {}
