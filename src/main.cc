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
#include <cmath>
#include <cstdio>
#include <PQP/PQP.h>
#include <Eigen/Dense>
#include <iostream>

#define LISTS 1

const double seg_width = 0.0001;
typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix;
typedef Eigen::Vector3f Vector;

void DHTable(double theta, double d, double a, double alpha,
             Matrix& R, Vector& T) {
  R << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),
       sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),
       0.0,         sin(alpha),             cos(alpha);

  T << a*cos(theta), a*sin(theta), d;
}

int main() {
  // initialize PQP model pointers
  PQP_Model *b1 = new PQP_Model;
  PQP_Model *b2 = new PQP_Model;
  PQP_Model *obs = new PQP_Model;

  b1->BeginModel();
  b2->BeginModel();
  obs->BeginModel();
  PQP_REAL tri1[3][3] {{0.0, 0.0, 0.0},
    {1.5, 0.0, 0.0},
    {1.5, seg_width, 0.0}
  },

  tri2[3][3] {{0.0, 0.0, 0.0},
    {0.0, seg_width, 0.0},
    {1.5, seg_width, 0.0}
  },

  tri3[3][3] {{0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {1.0, seg_width, 0.0}
  },

  tri4[3][3] {{0.0, 0.0, 0.0},
    {0.0, seg_width, 0.0},
    {1.0, seg_width, 0.0}
  },

  tri5[3][3] {{-0.999, 1.5, 0.0},
    {-1.0, 1.501, 0.0},
    {-1.0, 1.499, 0.0}
  };

  b1->AddTri(tri1[0], tri1[1], tri1[2], 0);
  b1->AddTri(tri2[0], tri2[1], tri2[2], 1);
  b2->AddTri(tri3[0], tri3[1], tri3[2], 0);
  b2->AddTri(tri4[0], tri4[1], tri4[2], 1);
  obs->AddTri(tri5[0], tri5[1], tri5[2], 0);

  b1->EndModel();
  b2->EndModel();
  obs->EndModel();
  Matrix dh1R, dh2R;
  Vector dh1T, dh2T;

  DHTable(0.0, 0.0, 1.5, 0.0, dh1R, dh1T);
  DHTable(0.0, 0.0, 1.0, 0.0, dh2R, dh2T);

  Matrix R1, R2, R = Matrix::Identity();

  double q1 = M_PI_2, q2 = 0;
  R1 << cos(q1), -sin(q1), 0.0,
        sin(q1),  cos(q1), 0.0,
        0.0,          0.0, 1.0;

  R2 << cos(q2), -sin(q2), 0.0,
        sin(q2),  cos(q2), 0.0,
        0.0,          0.0, 1.0;

  Vector T1, T2, T;
  T <<  0.0, 0.0, 0.0;
  T1 << 0.0, 0.0, 0.0;
  T2 << 0.0, 0.0, 0.0;

  R2 = R1 * dh1R * R2;
  T2 = R2 * T2 + R1 * dh1T;

  Vector v;
  v << 0.0, 0.0, 0.0;
  std::cout << R1 * v << std::endl;
  std::cout << R2 * v + R1 * dh1T << std::endl;

  // perform a collision query

  PQP_CollideResult cres;
  PQP_Collide(&cres, reinterpret_cast<PQP_REAL(*)[3]>(R1.data()), T1.data(), b1,
              reinterpret_cast<PQP_REAL(*)[3]>(R.data()), T.data(),
              obs, PQP_ALL_CONTACTS);

  // looking at the report, we can see where all the contacts were, and
  // also how many tests were necessary:

  printf("\nAll contact collision query between overlapping tori:\n");
  printf("Num BV tests: %d\n", cres.NumBVTests());
  printf("Num Tri tests: %d\n", cres.NumTriTests());
  printf("Num contact pairs: %d\n", cres.NumPairs());
#if LISTS
  int i;
  for(i=0; i<cres.NumPairs(); i++) {
    printf("\t contact %4d: tri %4d and tri %4d\n",
           i,
           cres.Id1(i),
           cres.Id2(i));
  }
#endif

  PQP_Collide(&cres, reinterpret_cast<PQP_REAL(*)[3]>(R2.data()), T2.data(), b2,
              reinterpret_cast<PQP_REAL(*)[3]>(R.data()), T.data(),
              obs, PQP_ALL_CONTACTS);

  // looking at the report, we can see where all the contacts were, and
  // also how many tests were necessary:

  printf("\nAll contact collision query between overlapping tori:\n");
  printf("Num BV tests: %d\n", cres.NumBVTests());
  printf("Num Tri tests: %d\n", cres.NumTriTests());
  printf("Num contact pairs: %d\n", cres.NumPairs());
#if LISTS

  for(i=0; i<cres.NumPairs(); i++) {
    printf("\t contact %4d: tri %4d and tri %4d\n",
           i,
           cres.Id1(i),
           cres.Id2(i));
  }
#endif

  // Notice the PQP_ALL_CONTACTS flag we used in the call to PQP_Collide.
  // The alternative is to use the PQP_FIRST_CONTACT flag, instead.
  // The result is that the collide routine searches for any contact,
  // but not all of them.  It can take many many fewer tests to locate a single
  // contact.

  // Perform a distance query, which should return a distance of 0.0

  PQP_DistanceResult dres;
  PQP_Distance(&dres, reinterpret_cast<PQP_REAL(*)[3]>(R2.data()), T2.data(),
               b2, reinterpret_cast<PQP_REAL(*)[3]>(R.data()), T.data(), obs,
               0.0, 0.0);

  printf("\nDistance query between overlapping tori\n");
  printf("Num BV tests: %d\n", dres.NumBVTests());
  printf("Num Tri tests: %d\n", dres.NumTriTests());
  printf("Distance: %lf\n", dres.Distance());

  delete b1;
  delete b2;
  delete obs;

  return 0;
}

