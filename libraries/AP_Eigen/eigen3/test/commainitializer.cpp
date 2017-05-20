// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "main.h"

void test_commainitializer()
{
  Matrix3d m3;
  Matrix4d m4;

  VERIFY_RAISES_ASSERT( (m3 << 1, 2, 3, 4, 5, 6, 7, 8) );
  
  #ifndef _MSC_VER
  VERIFY_RAISES_ASSERT( (m3 << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10) );
  #endif

  double data[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  Matrix3d ref = Map<Matrix<double,3,3,RowMajor> >(data);

  m3 = Matrix3d::Random();
  m3 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  VERIFY_IS_APPROX(m3, ref );

  Vector3d vec[3];
  vec[0] << 1, 4, 7;
  vec[1] << 2, 5, 8;
  vec[2] << 3, 6, 9;
  m3 = Matrix3d::Random();
  m3 << vec[0], vec[1], vec[2];
  VERIFY_IS_APPROX(m3, ref);

  vec[0] << 1, 2, 3;
  vec[1] << 4, 5, 6;
  vec[2] << 7, 8, 9;
  m3 = Matrix3d::Random();
  m3 << vec[0].transpose(),
        4, 5, 6,
        vec[2].transpose();
  VERIFY_IS_APPROX(m3, ref);
}
