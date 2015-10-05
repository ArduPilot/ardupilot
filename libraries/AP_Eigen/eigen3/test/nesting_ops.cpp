// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2010 Hauke Heibel <hauke.heibel@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "main.h"

template <typename MatrixType> void run_nesting_ops(const MatrixType& _m)
{
  typename internal::nested_eval<MatrixType,2>::type m(_m);

  // Make really sure that we are in debug mode!
  VERIFY_RAISES_ASSERT(eigen_assert(false));

  // The only intention of these tests is to ensure that this code does
  // not trigger any asserts or segmentation faults... more to come.
  VERIFY_IS_APPROX( (m.transpose() * m).diagonal().sum(), (m.transpose() * m).diagonal().sum() );
  VERIFY_IS_APPROX( (m.transpose() * m).diagonal().array().abs().sum(), (m.transpose() * m).diagonal().array().abs().sum() );

  VERIFY_IS_APPROX( (m.transpose() * m).array().abs().sum(), (m.transpose() * m).array().abs().sum() );
}

void test_nesting_ops()
{
  CALL_SUBTEST_1(run_nesting_ops(MatrixXf::Random(25,25)));
  CALL_SUBTEST_2(run_nesting_ops(MatrixXd::Random(25,25)));
  CALL_SUBTEST_3(run_nesting_ops(Matrix4f::Random()));
  CALL_SUBTEST_4(run_nesting_ops(Matrix4d::Random()));
}
