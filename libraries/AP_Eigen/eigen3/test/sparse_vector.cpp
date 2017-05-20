// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2011 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "sparse.h"

template<typename Scalar,typename Index> void sparse_vector(int rows, int cols)
{
  double densityMat = (std::max)(8./(rows*cols), 0.01);
  double densityVec = (std::max)(8./float(rows), 0.1);
  typedef Matrix<Scalar,Dynamic,Dynamic> DenseMatrix;
  typedef Matrix<Scalar,Dynamic,1> DenseVector;
  typedef SparseVector<Scalar,0,Index> SparseVectorType;
  typedef SparseMatrix<Scalar,0,Index> SparseMatrixType;
  Scalar eps = 1e-6;

  SparseMatrixType m1(rows,rows);
  SparseVectorType v1(rows), v2(rows), v3(rows);
  DenseMatrix refM1 = DenseMatrix::Zero(rows, rows);
  DenseVector refV1 = DenseVector::Random(rows),
              refV2 = DenseVector::Random(rows),
              refV3 = DenseVector::Random(rows);

  std::vector<int> zerocoords, nonzerocoords;
  initSparse<Scalar>(densityVec, refV1, v1, &zerocoords, &nonzerocoords);
  initSparse<Scalar>(densityMat, refM1, m1);

  initSparse<Scalar>(densityVec, refV2, v2);
  initSparse<Scalar>(densityVec, refV3, v3);

  Scalar s1 = internal::random<Scalar>();

  // test coeff and coeffRef
  for (unsigned int i=0; i<zerocoords.size(); ++i)
  {
    VERIFY_IS_MUCH_SMALLER_THAN( v1.coeff(zerocoords[i]), eps );
    //VERIFY_RAISES_ASSERT( v1.coeffRef(zerocoords[i]) = 5 );
  }
  {
    VERIFY(int(nonzerocoords.size()) == v1.nonZeros());
    int j=0;
    for (typename SparseVectorType::InnerIterator it(v1); it; ++it,++j)
    {
      VERIFY(nonzerocoords[j]==it.index());
      VERIFY(it.value()==v1.coeff(it.index()));
      VERIFY(it.value()==refV1.coeff(it.index()));
    }
  }
  VERIFY_IS_APPROX(v1, refV1);
  
  // test coeffRef with reallocation
  {
    SparseVectorType v4(rows);
    DenseVector v5 = DenseVector::Zero(rows);
    for(int k=0; k<rows; ++k)
    {
      int i = internal::random<int>(0,rows-1);
      Scalar v = internal::random<Scalar>();
      v4.coeffRef(i) += v;
      v5.coeffRef(i) += v;
    }
    VERIFY_IS_APPROX(v4,v5);
  }

  v1.coeffRef(nonzerocoords[0]) = Scalar(5);
  refV1.coeffRef(nonzerocoords[0]) = Scalar(5);
  VERIFY_IS_APPROX(v1, refV1);

  VERIFY_IS_APPROX(v1+v2, refV1+refV2);
  VERIFY_IS_APPROX(v1+v2+v3, refV1+refV2+refV3);

  VERIFY_IS_APPROX(v1*s1-v2, refV1*s1-refV2);

  VERIFY_IS_APPROX(v1*=s1, refV1*=s1);
  VERIFY_IS_APPROX(v1/=s1, refV1/=s1);

  VERIFY_IS_APPROX(v1+=v2, refV1+=refV2);
  VERIFY_IS_APPROX(v1-=v2, refV1-=refV2);

  VERIFY_IS_APPROX(v1.dot(v2), refV1.dot(refV2));
  VERIFY_IS_APPROX(v1.dot(refV2), refV1.dot(refV2));

  VERIFY_IS_APPROX(m1*v2, refM1*refV2);
  VERIFY_IS_APPROX(v1.dot(m1*v2), refV1.dot(refM1*refV2));
  int i = internal::random<int>(0,rows-1);
  VERIFY_IS_APPROX(v1.dot(m1.col(i)), refV1.dot(refM1.col(i)));


  VERIFY_IS_APPROX(v1.squaredNorm(), refV1.squaredNorm());
  
  VERIFY_IS_APPROX(v1.blueNorm(), refV1.blueNorm());

  // test aliasing
  VERIFY_IS_APPROX((v1 = -v1), (refV1 = -refV1));
  VERIFY_IS_APPROX((v1 = v1.transpose()), (refV1 = refV1.transpose().eval()));
  VERIFY_IS_APPROX((v1 += -v1), (refV1 += -refV1));
  
  // sparse matrix to sparse vector
  SparseMatrixType mv1;
  VERIFY_IS_APPROX((mv1=v1),v1);
  VERIFY_IS_APPROX(mv1,(v1=mv1));
  VERIFY_IS_APPROX(mv1,(v1=mv1.transpose()));
  
  // check copy to dense vector with transpose
  refV3.resize(0);
  VERIFY_IS_APPROX(refV3 = v1.transpose(),v1.toDense()); 
  VERIFY_IS_APPROX(DenseVector(v1),v1.toDense()); 

}

void test_sparse_vector()
{
  for(int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(( sparse_vector<double,int>(8, 8) ));
    CALL_SUBTEST_2(( sparse_vector<std::complex<double>, int>(16, 16) ));
    CALL_SUBTEST_1(( sparse_vector<double,long int>(299, 535) ));
    CALL_SUBTEST_1(( sparse_vector<double,short>(299, 535) ));
  }
}

