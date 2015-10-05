// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2010-2011 Jitse Niesen <jitse@maths.leeds.ac.uk>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "main.h"

template<typename MatrixType>
bool equalsIdentity(const MatrixType& A)
{
  typedef typename MatrixType::Scalar Scalar;
  Scalar zero = static_cast<Scalar>(0);

  bool offDiagOK = true;
  for (Index i = 0; i < A.rows(); ++i) {
    for (Index j = i+1; j < A.cols(); ++j) {
      offDiagOK = offDiagOK && (A(i,j) == zero);
    }
  }
  for (Index i = 0; i < A.rows(); ++i) {
    for (Index j = 0; j < (std::min)(i, A.cols()); ++j) {
      offDiagOK = offDiagOK && (A(i,j) == zero);
    }
  }

  bool diagOK = (A.diagonal().array() == 1).all();
  return offDiagOK && diagOK;
}

template<typename VectorType>
void testVectorType(const VectorType& base)
{
  typedef typename VectorType::Scalar Scalar;

  const Index size = base.size();
  
  Scalar high = internal::random<Scalar>(-500,500);
  Scalar low = (size == 1 ? high : internal::random<Scalar>(-500,500));
  if (low>high) std::swap(low,high);

  const Scalar step = ((size == 1) ? 1 : (high-low)/(size-1));

  // check whether the result yields what we expect it to do
  VectorType m(base);
  m.setLinSpaced(size,low,high);

  VectorType n(size);
  for (int i=0; i<size; ++i)
    n(i) = low+i*step;

  VERIFY_IS_APPROX(m,n);

  // random access version
  m = VectorType::LinSpaced(size,low,high);
  VERIFY_IS_APPROX(m,n);

  // Assignment of a RowVectorXd to a MatrixXd (regression test for bug #79).
  VERIFY( (MatrixXd(RowVectorXd::LinSpaced(3, 0, 1)) - RowVector3d(0, 0.5, 1)).norm() < std::numeric_limits<Scalar>::epsilon() );

  // These guys sometimes fail! This is not good. Any ideas how to fix them!?
  //VERIFY( m(m.size()-1) == high );
  //VERIFY( m(0) == low );

  // sequential access version
  m = VectorType::LinSpaced(Sequential,size,low,high);
  VERIFY_IS_APPROX(m,n);

  // These guys sometimes fail! This is not good. Any ideas how to fix them!?
  //VERIFY( m(m.size()-1) == high );
  //VERIFY( m(0) == low );

  // check whether everything works with row and col major vectors
  Matrix<Scalar,Dynamic,1> row_vector(size);
  Matrix<Scalar,1,Dynamic> col_vector(size);
  row_vector.setLinSpaced(size,low,high);
  col_vector.setLinSpaced(size,low,high);
  // when using the extended precision (e.g., FPU) the relative error might exceed 1 bit
  // when computing the squared sum in isApprox, thus the 2x factor.
  VERIFY( row_vector.isApprox(col_vector.transpose(), Scalar(2)*NumTraits<Scalar>::epsilon()));

  Matrix<Scalar,Dynamic,1> size_changer(size+50);
  size_changer.setLinSpaced(size,low,high);
  VERIFY( size_changer.size() == size );

  typedef Matrix<Scalar,1,1> ScalarMatrix;
  ScalarMatrix scalar;
  scalar.setLinSpaced(1,low,high);
  VERIFY_IS_APPROX( scalar, ScalarMatrix::Constant(high) );
  VERIFY_IS_APPROX( ScalarMatrix::LinSpaced(1,low,high), ScalarMatrix::Constant(high) );

  // regression test for bug 526 (linear vectorized transversal)
  if (size > 1) {
    m.tail(size-1).setLinSpaced(low, high);
    VERIFY_IS_APPROX(m(size-1), high);
  }
}

template<typename MatrixType>
void testMatrixType(const MatrixType& m)
{
  const Index rows = m.rows();
  const Index cols = m.cols();

  MatrixType A;
  A.setIdentity(rows, cols);
  VERIFY(equalsIdentity(A));
  VERIFY(equalsIdentity(MatrixType::Identity(rows, cols)));
}

void test_nullary()
{
  CALL_SUBTEST_1( testMatrixType(Matrix2d()) );
  CALL_SUBTEST_2( testMatrixType(MatrixXcf(internal::random<int>(1,300),internal::random<int>(1,300))) );
  CALL_SUBTEST_3( testMatrixType(MatrixXf(internal::random<int>(1,300),internal::random<int>(1,300))) );
  
  for(int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_4( testVectorType(VectorXd(internal::random<int>(1,300))) );
    CALL_SUBTEST_5( testVectorType(Vector4d()) );  // regression test for bug 232
    CALL_SUBTEST_6( testVectorType(Vector3d()) );
    CALL_SUBTEST_7( testVectorType(VectorXf(internal::random<int>(1,300))) );
    CALL_SUBTEST_8( testVectorType(Vector3f()) );
    CALL_SUBTEST_8( testVectorType(Matrix<float,1,1>()) );
  }
}
