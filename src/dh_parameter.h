#ifndef DH_PARAMETER_H_INCLUDED
#define DH_PARAMETER_H_INCLUDED

#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <PQP/PQP.h>
#include <Eigen/Dense>


// Check to place these typedefs in a namespace, or in the class definition
// because it is also used in pqp environment
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> EMatrix;
typedef Eigen::Vector3d EVector;
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

private:
  EMatrix rotation_;
  EVector translation_;
};

#endif // DH_PARAMETER_H_INCLUDED
