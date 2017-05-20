// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2011 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "main.h"


template<typename MatrixType> void zeroReduction(const MatrixType& m) {
  // Reductions that must hold for zero sized objects
  VERIFY(m.all());
  VERIFY(!m.any());
  VERIFY(m.prod()==1);
  VERIFY(m.sum()==0);
  VERIFY(m.count()==0);
  VERIFY(m.allFinite());
  VERIFY(!m.hasNaN());
}


template<typename MatrixType> void zeroSizedMatrix()
{
  MatrixType t1;

  if (MatrixType::SizeAtCompileTime == Dynamic || MatrixType::SizeAtCompileTime == 0)
  {
    zeroReduction(t1);
    if (MatrixType::RowsAtCompileTime == Dynamic)
      VERIFY(t1.rows() == 0);
    if (MatrixType::ColsAtCompileTime == Dynamic)
      VERIFY(t1.cols() == 0);

    if (MatrixType::RowsAtCompileTime == Dynamic && MatrixType::ColsAtCompileTime == Dynamic)
    {

      MatrixType t2(0, 0);
      VERIFY(t2.rows() == 0);
      VERIFY(t2.cols() == 0);

      zeroReduction(t2);
      VERIFY(t1==t2);
    }
  }
}

template<typename VectorType> void zeroSizedVector()
{
  VectorType t1;

  if (VectorType::SizeAtCompileTime == Dynamic || VectorType::SizeAtCompileTime==0)
  {
    zeroReduction(t1);
    VERIFY(t1.size() == 0);
    VectorType t2(DenseIndex(0)); // DenseIndex disambiguates with 0-the-null-pointer (error with gcc 4.4 and MSVC8)
    VERIFY(t2.size() == 0);
    zeroReduction(t2);

    VERIFY(t1==t2);
  }
}

void test_zerosized()
{
  zeroSizedMatrix<Matrix2d>();
  zeroSizedMatrix<Matrix3i>();
  zeroSizedMatrix<Matrix<float, 2, Dynamic> >();
  zeroSizedMatrix<MatrixXf>();
  zeroSizedMatrix<Matrix<float, 0, 0> >();
  zeroSizedMatrix<Matrix<float, Dynamic, 0, 0, 0, 0> >();
  zeroSizedMatrix<Matrix<float, 0, Dynamic, 0, 0, 0> >();
  zeroSizedMatrix<Matrix<float, Dynamic, Dynamic, 0, 0, 0> >();
  zeroSizedMatrix<Matrix<float, 0, 4> >();
  zeroSizedMatrix<Matrix<float, 4, 0> >();

  zeroSizedVector<Vector2d>();
  zeroSizedVector<Vector3i>();
  zeroSizedVector<VectorXf>();
  zeroSizedVector<Matrix<float, 0, 1> >();
  zeroSizedVector<Matrix<float, 1, 0> >();
}
