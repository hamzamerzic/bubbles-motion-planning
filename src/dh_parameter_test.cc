#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE DhTest

#include "dh_parameter.h"
#include <cmath>
#include <vector>

#include <boost/test/unit_test.hpp>

void CheckCloseTables(const std::vector<std::vector<float>>& vec_table_rot,
                      const std::vector<float>& vec_table_trans,
                      const DhParameter& dh_table, double eps = 0.1) {
  const double small = 0.0001;
  for(int i(0); i < 3; ++i)
    for(int j(0); j < 3; ++j) {
      double expected (vec_table_rot[i][j]),
             returned (dh_table.rotation()(i, j));
      if(std::fabs(expected) < small)
        BOOST_CHECK_SMALL(returned, small);
      else
        BOOST_CHECK_CLOSE(expected, returned, eps);
    }

  for(int i(0); i < 3; ++i) {
      double expected(vec_table_trans[i]),
             returned(static_cast<double>(dh_table.translation()[i]));
      if(std::fabs(expected) < small)
        BOOST_CHECK_SMALL(returned, small);
      else
        BOOST_CHECK_CLOSE(expected, returned, eps);
  }
}

void CheckCloseTransform(const EMatrix& R, const EVector& T,
                         const EMatrix& R_exp, const EVector& T_exp,
                         double eps = 0.1) {
  const double small = 0.0001;
  for(int i(0); i < 3; ++i)
    for(int j(0); j < 3; ++j) {
      double expected (R_exp(i, j)), returned (R(i, j));
      if(std::fabs(expected) < small)
        BOOST_CHECK_SMALL(returned, small);
      else
        BOOST_CHECK_CLOSE(expected, returned, eps);
    }

  for(int i(0); i < 3; ++i) {
      double expected (T_exp(i)), returned (T(i));
      if(std::fabs(expected) < small)
        BOOST_CHECK_SMALL(returned, small);
      else
        BOOST_CHECK_CLOSE(expected, returned, eps);
  }
}

BOOST_AUTO_TEST_CASE(constructor1) {
  const double pi_6 = M_PI / 6;
  CheckCloseTables({{0.866, 0.0,    0.5},
                    {  0.5, 0.0, -0.866},
                    {  0.0, 1.0,    0.0}},
                   {0.217, 0.125, 0},
                   DhParameter(pi_6, 0, 0.25, M_PI_2), 1.0);
}

BOOST_AUTO_TEST_CASE(constructor2) {
  const double pi_6 = M_PI / 6, pi_5 = M_PI / 5;
  CheckCloseTables({{0.866, -0.404,  0.293},
                    {  0.5,    0.7, -0.509},
                    {  0.0,  0.587,  0.809}},
                   {0.866, 0.5, 2.0},
                   DhParameter(pi_6, 2, 1, pi_5), 1.0);
}

BOOST_AUTO_TEST_CASE(transform) {
  const double pi_6 = M_PI / 6, pi_5 = M_PI / 5;
  DhParameter dh (pi_6, 2, 1, pi_5);
  EMatrix R (EMatrix::Identity());
  EVector T (1, 2, 3);
  dh.Transform(R, T);

  EMatrix R_exp;
  R_exp << 0.86603, -0.40451,   0.29389,
           0.50000,  0.70063,  -0.50904,
           0.00000,  0.58779,   0.80902;
  EVector T_exp (1.8660, 2.5, 5.0);
  CheckCloseTransform(R, T, R_exp, T_exp);
}