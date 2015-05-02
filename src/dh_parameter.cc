/*
 * Copyright (C) 2015 Hamza Merzić
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "dh_parameter.h"

#include <cmath>

DhParameter::DhParameter(const double theta, const double d,
                         const double a, const double alpha)
    : translation_(a*cos(theta), a*sin(theta), d) {

  rotation_ << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha),
               sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha),
                      0.0,               sin(alpha),               cos(alpha);
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
