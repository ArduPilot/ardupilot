// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2012 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "main.h"
#include <limits>
#include <Eigen/Eigenvalues>

template<typename MatrixType> void generalized_eigensolver_real(const MatrixType& m)
{
  typedef typename MatrixType::Index Index;
  /* this test covers the following files:
     GeneralizedEigenSolver.h
  */
  Index rows = m.rows();
  Index cols = m.cols();

  typedef typename MatrixType::Scalar Scalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;

  MatrixType a = MatrixType::Random(rows,cols);
  MatrixType b = MatrixType::Random(rows,cols);
  MatrixType a1 = MatrixType::Random(rows,cols);
  MatrixType b1 = MatrixType::Random(rows,cols);
  MatrixType spdA =  a.adjoint() * a + a1.adjoint() * a1;
  MatrixType spdB =  b.adjoint() * b + b1.adjoint() * b1;

  // lets compare to GeneralizedSelfAdjointEigenSolver
  GeneralizedSelfAdjointEigenSolver<MatrixType> symmEig(spdA, spdB);
  GeneralizedEigenSolver<MatrixType> eig(spdA, spdB);

  VERIFY_IS_EQUAL(eig.eigenvalues().imag().cwiseAbs().maxCoeff(), 0);

  VectorType realEigenvalues = eig.eigenvalues().real();
  std::sort(realEigenvalues.data(), realEigenvalues.data()+realEigenvalues.size());
  VERIFY_IS_APPROX(realEigenvalues, symmEig.eigenvalues());
}

void test_eigensolver_generalized_real()
{
  for(int i = 0; i < g_repeat; i++) {
    int s = 0;
    CALL_SUBTEST_1( generalized_eigensolver_real(Matrix4f()) );
    s = internal::random<int>(1,EIGEN_TEST_MAX_SIZE/4);
    CALL_SUBTEST_2( generalized_eigensolver_real(MatrixXd(s,s)) );

    // some trivial but implementation-wise tricky cases
    CALL_SUBTEST_2( generalized_eigensolver_real(MatrixXd(1,1)) );
    CALL_SUBTEST_2( generalized_eigensolver_real(MatrixXd(2,2)) );
    CALL_SUBTEST_3( generalized_eigensolver_real(Matrix<double,1,1>()) );
    CALL_SUBTEST_4( generalized_eigensolver_real(Matrix2d()) );
    TEST_SET_BUT_UNUSED_VARIABLE(s)
  }
}
