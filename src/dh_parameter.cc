#include "dh_parameter.h"

#include <cmath>

DhParameter::DhParameter(const double theta,
                         const double d,
                         const double a,
                         const double alpha):
  translation_(a*cos(theta), a*sin(theta), d) {

  rotation_ << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),
               sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),
                      0.0,             sin(alpha),             cos(alpha);
}

const PqpQueryType* DhParameter::RotData() {
  return reinterpret_cast<PQP_REAL(*)[3]>(rotation_.data());
}

const PQP_REAL* DhParameter::TransData() {
  return translation_.data();
}

const void DhParameter::Transform(EMatrix& R, EVector& T) const {
  T = R * translation_ + T;
  R = R * rotation_;
}

const void DhParameter::InverseTransform(EMatrix& R, EVector& T) const {
  T = rotation_.transpose() * T - rotation_.transpose() * translation_;
  R = rotation_.transpose() * R;
}
