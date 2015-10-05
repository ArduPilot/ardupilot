// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#define EIGEN_NO_STATIC_ASSERT
#include "product.h"
#include <Eigen/LU>

// regression test for bug 447
void product1x1()
{
  Matrix<float,1,3> matAstatic;
  Matrix<float,3,1> matBstatic;
  matAstatic.setRandom();
  matBstatic.setRandom();
  VERIFY_IS_APPROX( (matAstatic * matBstatic).coeff(0,0), 
                    matAstatic.cwiseProduct(matBstatic.transpose()).sum() );

  MatrixXf matAdynamic(1,3);
  MatrixXf matBdynamic(3,1);
  matAdynamic.setRandom();
  matBdynamic.setRandom();
  VERIFY_IS_APPROX( (matAdynamic * matBdynamic).coeff(0,0), 
                    matAdynamic.cwiseProduct(matBdynamic.transpose()).sum() );
}


void test_product_small()
{
  for(int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1( product(Matrix<float, 3, 2>()) );
    CALL_SUBTEST_2( product(Matrix<int, 3, 5>()) );
    CALL_SUBTEST_3( product(Matrix3d()) );
    CALL_SUBTEST_4( product(Matrix4d()) );
    CALL_SUBTEST_5( product(Matrix4f()) );
    CALL_SUBTEST_6( product1x1() );
  }

#ifdef EIGEN_TEST_PART_6
  {
    // test compilation of (outer_product) * vector
    Vector3f v = Vector3f::Random();
    VERIFY_IS_APPROX( (v * v.transpose()) * v, (v * v.transpose()).eval() * v);
  }
  
  {
    // regression test for pull-request #93
    Eigen::Matrix<double, 1, 1> A;  A.setRandom();
    Eigen::Matrix<double, 18, 1> B; B.setRandom();
    Eigen::Matrix<double, 1, 18> C; C.setRandom();
    VERIFY_IS_APPROX(B * A.inverse(), B * A.inverse()[0]);
    VERIFY_IS_APPROX(A.inverse() * C, A.inverse()[0] * C);
  }
#endif
}
