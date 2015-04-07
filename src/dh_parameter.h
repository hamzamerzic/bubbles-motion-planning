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
#ifndef DH_PARAMETER_H_INCLUDED
#define DH_PARAMETER_H_INCLUDED

#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <PQP/PQP.h>
#include <Eigen/Dense>


// TODO: Check to place these typedefs in a namespace, or in the class
// definition because it is also used in pqp environment
typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> EMatrix;
typedef Eigen::Vector3f EVector;
typedef PQP_REAL PqpQueryType[3];   //Return type for cleaner code
                                    //Needed for PQP query

class DhParameter {
public:
  DhParameter(const double theta, const double d,
              const double a, const double alpha);

  const EMatrix& rotation() const { return rotation_; }
  const EVector& translation() const { return translation_; }
  EMatrix& rotation() { return rotation_; }
  EVector& translation() { return translation_; }

  //Returns pointers for PQP queries
  const PqpQueryType* RotData();
  const PQP_REAL* TransData();

  // Applies DH transform to input matrices
  const void Transform(EMatrix& R, EVector& T) const;

  // Used for placing a segment to origin
  const void InverseTransform(EMatrix& R, EVector& T) const;

private:
  EMatrix rotation_;
  EVector translation_;
};

#endif // DH_PARAMETER_H_INCLUDED
